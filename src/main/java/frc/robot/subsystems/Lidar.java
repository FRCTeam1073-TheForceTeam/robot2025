package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Scan;

public class Lidar extends SubsystemBase {
    SerialPort serialPort;
    ArrayList <Scan> one = new ArrayList<Scan>();
    int one_count = 0; // Keep separate count of scans to avoid reallocation
    ArrayList <Scan> two = new ArrayList<Scan>();
    int two_count = 0; // Keep separate count of scans to avoid reallocation


    byte getInfo[] = {(byte) 0xa5, (byte) 0x52};
    byte scanDescriptor[] = {(byte) 0xa5, (byte) 0x5a, (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x40, (byte) 0x81};
    byte stopCommand[] = {(byte) 0xa5, (byte) 0x25};
    byte startCommand[] = {(byte) 0xa5, (byte) 0x20};
    boolean arrayTwoFilled = false;
    boolean recordScan = true;
    boolean measureMode = false;
    boolean writeToOne = true;
    double arrayOneTimestamp;
    double arrayTwoTimestamp;
    double timestamp = 0.0;
    double x_l;
    double y_l;
    double sumx = 0;
    double sumy = 0;
    double sumxx = 0;
    double sumyy = 0;
    double sumxy = 0;
    double xbar = 0;
    double ybar = 0;
    double varx = 0;
    double vary = 0;
    double covxy = 0;
    double sumvars = 0;
    double diffvars = 0;
    double discriminant = 0;
    double sqrtdiscr = 0;
    double lambdaplus, lambdaminus;
    double aplus, bplus, aminus, bminus;
    double aParallel, aNormal, bParallel, bNormal;
    double k = 2;
    double mag = 0;
    double denomPlus, denomMinus, majoraxis, minoraxis;
    double distance;
    double ave;
    double lastAverage = 2;
    double lastSlope = 5;
    double s;
    double ssum;
    double slope;
    float angle_deg;
    float angle_rad;
    float quality;
    float range_m;
    float range_mm;
    final double minAcceptedRange = 0.09; // In meters
    final double maxAcceptedRange = 2.2; // In meters
    double maxAcceptedX = 1.0;
    double minAcceptedY = -0.39;
    double maxAcceptedY = 0.39;
    final int minAcceptedAngle1 = 0; // In degrees, for the first range of accepted angles
    final int maxAcceptedAngle1 = 45; // In degrees, for the first range of accepted angles
    final int minAcceptedAngle2 = 325; // In degrees, for the second range of accepted angles
    final int maxAcceptedAngle2 = 360; // In degrees, for the second range of accepted angles
    private final int minAcceptedQuality = 5;
    private final int bytesPerScan = 5;
    int numBytesAvail = 0;
    int numTimesLidarArraySwitch = 0;
    int numScansRead;
    int numScansToRead;
    int count;
    int offset;
    int counter = 0;
    int sign = 0;
    int timesBad = 0;
    int meanXCounter = 0;
    Transform2d robotToLidar = new Transform2d(new Translation2d(0.289, 0.22), new Rotation2d()); // Translation needs x and y, rotation needs 
    Matrix<N3,N3> T;

    public Lidar () {
        T = robotToLidar.toMatrix();
        one.ensureCapacity(512);
        two.ensureCapacity(512);

        // Add all our scan records *once* and reuse them later.
        for (int ii = 0; ii < 512; ++ii) {
            one.add(new Scan(0,0,0,0,0));
            two.add(new Scan(0,0,0,0,0));
        }

        try{
            serialPort = new SerialPort(460800, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        }
        catch(Exception e){
            System.out.println("No LiDAR found");
            serialPort = null;
        }
        if(serialPort != null){
            serialPort.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
            super.setSubsystem("Lidar");
            serialPort.setFlowControl(SerialPort.FlowControl.kNone);
            System.out.println("Lidar constructor");
            Handshake();
        }
    }


    public void Handshake () {
        System.out.println("Beginning of handshake method for Lidar sensor");

        //send stop
        serialPort.write(stopCommand, stopCommand.length);
        try {
            Thread.sleep(50);
        }
        catch (Exception e) {
            System.out.println(e);
        }
        serialPort.flush();
        serialPort.reset();
        // ?? add GET_HEALTH Request?
        //startCommand
        serialPort.write(startCommand, startCommand.length);
        System.out.println("Sent start command to Lidar sensor");
        // wait loop, while getBytesReceived is less, sleep
        while(serialPort.getBytesReceived() < 7 && counter < 15){
            try {
                Thread.sleep(50);
            }
            catch (Exception e) {
                System.out.println(e);
            }
            counter ++;
        }

        if(counter >= 15){
            serialPort = null;
        }
        
        // How to print out hex in java - String.format("0x%02x", what you're printing out)

    }

    // Returns one of the two double buffers, but the number of scans comes from getScanCount()
    public ArrayList<Scan> getLidarArray(){
        if(writeToOne){
            return two; 
        } else {
            return one;
        }
    }

    // Return a count to match the scan array. Use with getLidarArray() to access data.
    public int getScanCount() {
        if (writeToOne) {
            return two_count;
        } else {
            return one_count;
        }
    }

    public double getLidarArrayTimestamp() {
        // return the opposite that is being filled
        if (writeToOne) return arrayTwoTimestamp;
        else return arrayOneTimestamp;
    }

    public boolean parseDescriptor(){
        byte [] received = serialPort.read(7);
        return Arrays.equals(received, scanDescriptor);
    }

    public boolean isXInRange(double x){
        return x <= maxAcceptedX && x > 0;
    }

    public boolean isYInRange(double y){
        return y > minAcceptedY && y < maxAcceptedY;
    }

    public boolean isAngleGood(float angle){
        if((angle > minAcceptedAngle1 && angle < maxAcceptedAngle1) || (angle > minAcceptedAngle2 && angle < maxAcceptedAngle2)) return true;
        else return false;
    }

    public boolean isRangeGood(float range){
        if((range < maxAcceptedRange) && (range > minAcceptedRange)) return true;
        else return false;
    }

    // what is most efficient?
    public void readAndParseMeasurements(int numBytesAvail){

        // round down to determine number of full scans available
        numScansToRead = numBytesAvail/bytesPerScan;
        byte[] rawData = serialPort.read(numScansToRead * bytesPerScan);
        for(int i = 0; i < numScansToRead; i ++){
            offset = i * bytesPerScan;
            recordScan = true;
           if((rawData[offset] & 0x003) == 1){
                if (writeToOne){
                    //done writing to one, switching to writing to two - sets the timestamp for array two
                    two_count = 0; // Start over with the count of used entries  in arraylist two.
                    arrayTwoTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    writeToOne = false;
                    if(arrayTwoFilled && getLidarArray().size() > 20){
                        calculateOutput();
                    }
                }
                else {
                    //done writing to two, switching to writing to one - sets the timestamp for array one
                    one_count = 0; // Start over with the count of used entries in arraylist one.
                    arrayOneTimestamp = Timer.getFPGATimestamp();
                    numTimesLidarArraySwitch ++;
                    arrayTwoFilled = true;
                    writeToOne = true;
                    if(getLidarArray().size() > 20){
                        calculateOutput();
                    }
                }
            } 
            
            // divide by 4 to drop the lower two bits
            quality = ((rawData[offset + 0] & 0x0FC) >> 2);
            if(quality < minAcceptedQuality) recordScan = false;
            // angle = Math.pow(2, 7) * angle[14:7] + angle[6:0]
            angle_deg = ((Byte.toUnsignedInt(rawData[offset + 2]) & 0x0FF) << 7) | ((Byte.toUnsignedInt(rawData[offset + 1]) & 0x0FE) >> 1);
            angle_deg /= 64.0f;
            // range = Math.pow(2, 8) * distance[15:8] + distance[7:0]
            range_mm = ((rawData[offset + 4] & 0x0FF) << 8) | (rawData[offset + 3] & 0x0FF);
            angle_rad = 3.141592f * angle_deg / 180.0f;
            range_m = (range_mm / 4.0f) / 1000;

            x_l = Math.cos(-angle_rad) * range_m;
            y_l = Math.sin(-angle_rad) * range_m;
            // Matrix<N3, N1> lidarPoint = VecBuilder.fill(x_l, y_l, 1.0);
            // Trying an offset-only approach, less general, but faster.
            double x_robot = x_l + T.get(0,2);
            double y_robot = y_l + T.get(1,2);
            // Matrix<N3, N1> robotPoint = T.times(lidarPoint);
            // Quality, range, and angle filter
            if (isAngleGood(angle_deg) == false) recordScan = false;
            if (isRangeGood(range_m) == false) recordScan = false;
            if (!isXInRange(x_robot)) recordScan = false;
            if (!isYInRange(y_robot)) recordScan = false;

            if(recordScan) {
                if (writeToOne && one_count < one.size()){
                    var scan = one.get(one_count);
                    // Fill out existing scan:
                    scan.range = range_m;
                    scan.angle = angle_rad;
                    scan.quality = quality;
                    scan.x_robot = x_robot;
                    scan.y_robot = y_robot;
                    one_count++; // Count this scan for array one.
                } 
                else if (!writeToOne && two_count < two.size()){
                    var scan = two.get(two_count);
                    // Fill out existing scan.
                    scan.range = range_m;
                    scan.angle = angle_rad;
                    scan.quality = quality;
                    scan.x_robot = x_robot;
                    scan.y_robot = y_robot;
                    two_count++; // Count this scan for array two.
                }
            }
        }
    }

    public void calculateOutput(){
        ave = 0;
        count = 0;
        s = 0;
        ssum = 0;
        int numScans = getScanCount();
        if(numScans > 20){
            for(int i = 0; i < numScans; i++){
                ave += getLidarArray().get(i).getX();
            }
            ave /= getLidarArray().size();
            for(int i = 0; i < numScans - 3; i++){
                var scan = getLidarArray().get(i);
                var scan2 = getLidarArray().get(i+2);

                if(Math.abs(scan2.getY() - scan.getY()) > 0.0001){
                    s = (scan2.getX() - scan.getX()) / (scan2.getY() - scan.getY());
                    if(s < 5 && s > -5){
                        ssum += s;
                        count++;
                    }
                }
            }
            slope = ssum / count;
        }        
    }

    public double getAverageX(){
        return ave;
    }

    public double getAverageSlope(){
        return slope;
    }

    public double getAngleToRotate(){
        if(Math.abs(getAverageSlope()) >= 5){
            return Math.PI;
        }
        return Math.atan(getAverageSlope());
    }

    public int getNumberScansToRead(){
        return numScansToRead;
    }


    @Override
    public void periodic() {
        if(serialPort == null){
            return;
        }
        if(arrayTwoFilled && getLidarArray().size() > 0){
            SmartDashboard.putNumber("Lidar/Times Lidar Array Switched", getTimesArraySwitch());
            SmartDashboard.putNumber("Lidar/Range", getRange());
            SmartDashboard.putNumber("Lidar/Angle", getAngle());
            SmartDashboard.putNumber("Lidar/Quality", getQuality());
            SmartDashboard.putNumber("Lidar/LiDAR X Value", getXVal());
            SmartDashboard.putNumber("Lidar/LiDAR Y Value", getYVal());
            SmartDashboard.putNumber("Lidar/LiDAR R Value", getRVal());
            SmartDashboard.putNumber("Lidar/Number of Scans in LiDAR Array", getScanCount());
            SmartDashboard.putNumber("Lidar/Number of Scans to Read", getNumberScansToRead());
            SmartDashboard.putNumber("Lidar/Average Slope", getAverageSlope());
            SmartDashboard.putNumber("Lidar/Average X", getAverageX());
            SmartDashboard.putNumber("Lidar/Angle to Rotate", getAngleToRotate());
        }
        numBytesAvail = serialPort.getBytesReceived();
         if(measureMode){
                // read and parse all available scan data to read - fill array
                readAndParseMeasurements(numBytesAvail);
        } else if(numBytesAvail >= 7) {
                // read and check the first 7 bytes of response
                if(parseDescriptor()){
                    // expected descriptor received, switch to read data
                    measureMode = true;
                }
                else{
                    System.out.println("Lidar handshake error");
                }
        }
    }

    public int getTimesArraySwitch(){
        return numTimesLidarArraySwitch;
    }
    // only call these methods if the size of the current lidar array > 0
    public double getRange(){
        return getLidarArray().get(0).getRange();
    }

    public double getAngle(){
        return getLidarArray().get(0).getAngle();
    }
    
    public double getQuality(){
        return getLidarArray().get(0).getQuality();
    }

    public double getXVal(){
        return getLidarArray().get(0).getX();
    }

    public double getRVal(){
        return getLidarArray().get(0).getX() * Math.cos(getAngle());
    }

    public double getYVal(){
        return getLidarArray().get(0).getY();
    }

}