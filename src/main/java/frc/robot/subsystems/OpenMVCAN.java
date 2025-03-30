/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OpenMVCAN extends SubsystemBase {
  private CAN openmv;
  private CANData recvData;
  /// Mode feedback of the OpenMV
  private int mode = -1;

  /// Heartbeat frame counter of the OpenMV
  private int frameCounter = -1;
  private int missedHeartbeats = 0;
  private long lastHeartbeat = 0; // Timestamp of last heartbeat.

  /// Configuration of the OpenMV
  private int imageWidth = -1;
  private int imageHeight = -1;
  private int simpleTargetTracking = 0;
  private int lineSegmentTracking = 0;
  private int colorDetection = 0;
  private int advancedTargetTracking = 0;

  /// Counter of configuration updates
  private int configurationUpdates = 0;
  
  /// Internal loop counter to allow loop skipping:
  private int loopCounter = 0;

  /// Keep our device ID for logging:
  private int deviceId = -1;


  /**
   * Data returned for simple tracking.
   */
  public class Track {
    int cx = 0;
    int cy = 0;
    int vx = 0;
    int vy = 0;
    int type = 0;
    int quality = 0;
  }

  ArrayList<Track> tracks;


  /**
   * Creates a new OpenMV with a specific deviceID using our Miscellaneous type and manufacturer defaults.
   * 
   * This interface assumes a specific CANBus protocol design following the FRCCan model:
   *  - Each device has a collection of APIs with messages by index.
   *  - For our OpenMV we use API Class 1 for getting configuration feedback from the camera
   *    - Class 1 Index 0 - Camera sends its configuration to us once in a while.
   *         - Gives us current mode and which features camera has.
   *    - Class 1 Index 1 - Camera sends status (current resolution) to us once in a while.
   *         - Gives us current image resolution being used so we can scale results.
   *    - Class 1 Index 2 - Heartbeat packet sent on every frame.
   *         - Current mode and frame-counter so we know it is getting valid/fresh data.
   *    - Class 2 Simple Target Tracking Index is slot for number of targets.
   *         - Provides target cx,cy (packed 12 bits each), velocity, target type, target quality.
   *    - Class 3 Line Data for reporting detected lines. Index is slot for number of line.
   *         - Provides packed 12bit x,y for line ends, line type and quality.
   *    - Class 4 Color Blob data, Index is slot for number of targets.
   *         - TBD this is a little specific to an old game.
   *    - Class 5 Advanced Tracker Data, Index is slot for number of targets.
   *         - Provides packed 12bit cx, cy, 16bit area, target type, quality and skew
   *    - Class 6 Range Data, Index is range slot.
   *         - Provides 16bit range data + quality for a range sensor.
   */
  public OpenMVCAN(int deviceId) {
    this.deviceId = deviceId;
    openmv = new CAN(deviceId, 173, 10);
    recvData = new CANData();
    tracks = new ArrayList<Track>();

    // Support up to 4 tracks:
    for (int i=0; i < 4; ++i) {
        tracks.add(new Track());
    }
  }

  /**
   * Write a CANBus packet to the given API Index.
   * @param APIIndex
   * @param data - Must be 8 bytes or less in length.
   */
  public void write(int APIIndex, byte[] data) {
    if (data.length <= 8)
      openmv.writePacket(data, APIIndex);
  }

  /**
   * Utility function creates an API Index from an APIClass and index, packing them
   * into 10 bits as specified by FRC CAN standards.
   * @param apiClass - Upper 6 bits of API Index
   * @param index - Lower 4 bits of API Index
   * @return
   */
  public int apiIndex(int apiClass, int index) {
      return ((apiClass & 0x03f) <<4) | (index & 0x0f);
    }

    /**
     * Read a CANBus packet into the data argument from the given APIIndex.
     * @param APIIndex - The API Index to read
     * @param data - Updated with new CANBus data if the read returns true.
     * @return - True if a new packet was available for that index, false if not (and does not update data)
     */
  public boolean read(int APIIndex, CANData data) {
    return openmv.readPacketNew(APIIndex, data);
  }

    /**
     * Send a CANBus packet to request that the OpenMV camera change modes.
     * 
     * @param mode - The requested mode.
     */
  public void setMode(byte mode){
    byte[] message = new byte[1];
    message[0] = mode;
    write(apiIndex(1, 3), message);
  }

  /**
   * Returns the mode received as feedback from the OpenMV camera. 
   * @return  The mode that we have receiced from the OpenMV camera or -1 if no update seen.
   */
  public int getMode() {
    return mode;
  }

  /**
   * Return the latest frame counter received as heartbeat feedback from OpenMV Camera or -1 if
   * no update seen.
   * @return Frame counter or -1.
   */
  public int getFrameCounter() {
    return frameCounter;
  }

  /**
   * Returns the timestamp of the last heartbeat message from the OpenMV camera or 0 if never seen.
   * 
   * @return
   */
  public long getLastHeartbeat() {
    return lastHeartbeat;
  }

  /**
   * Reads the OpenMV API 1, Index 2 frame and parses it for reading heartbeat messages from the camera.
   * 
   * @return True if we got a heartbeat mesage, false if we did not.
   */
  public boolean readHeartbeat() {
    if(read(apiIndex(1, 2), recvData) == true && recvData.length == 5) {
      mode = recvData.data[0];
      int counterHi = recvData.data[1] & 0xFF;
      int counterLo = recvData.data[2] & 0xFF;
      frameCounter = (counterHi << 8) | counterLo;
      lastHeartbeat = recvData.timestamp;
      return true;
    }
    return false;
  }

  /**
   * Read the configuratioun API  API 1, Index 0 and parses it into our configuration info.
   * @return True if configuration was read, false if it was not read.
   */
  public boolean readConfig(){
    if(read(apiIndex(1,0), recvData) == true && recvData.length == 8) {
      mode = recvData.data[0];//mode
      // config[1] = recvData.data[1];
      simpleTargetTracking = recvData.data[2]; //simple target tracking
      lineSegmentTracking = recvData.data[3]; //Line Segment tracking
      colorDetection = recvData.data[4]; //color detection
      advancedTargetTracking = recvData.data[5]; //advanced target tracking
      // ?? = recvData.data[6] * 16;
      // ?? = recvData.data[7] * 16;
      configurationUpdates++; // Count this configuration update.
      return true;
    }
    return false;
  }

  /**
   * Read back the camera status from API 1, Index 1 and parse it into image sizes and
   * other properties.
   * @return True if the camera status was read, false if it was not read.
   */
  public boolean readCameraStatus() {
    if(read(apiIndex(1,1), recvData) == true && recvData.length == 8) {
      imageWidth = recvData.data[0] * 4;
      imageHeight = recvData.data[1] * 4;
      return true;
    }
    return false;
  }

  /**
   * Return the simple target tracking field.
   * @return -1 if never updated, 0 if there is no simple target tracking, > 0 for number of slots.
   */
  public int getSimpleTargetTrackingSlots() {
    if (configurationUpdates > 0)
      return simpleTargetTracking;
    else
      return -1;
  }

  /**
   * Return the line segment tracking field.
   * @return -1 if never updated, 0 if there is no line segment tracking, > 0 for number of line segment slots.
   */
  public int getLineSegmentTrackingSlots() {
    if (configurationUpdates > 0)
      return lineSegmentTracking;
    else
      return -1;
  }

  /**
   * Return the color detection field.
   * @return -1 if never updated, 0 if there is no color detection, > 0 if there is color detection.
   */
  public int getColorDetectionSlots() {
    if (configurationUpdates > 0)
      return colorDetection;
    else
      return -1;
  }

  /**
   * Return the advanced target tracking field.
   * @return -1 if never updated, 0 is there is no advanced target tracking, > 0 for number of advanced target slots.
   */
  public int getAdvancedTargetTrackingSlots() {
    if (configurationUpdates > 0)
      return advancedTargetTracking;
    else
      return -1;
  }

  /**
   * Return the current configured image width of the OpenMV camera.
   * @return Image width feedback from camera.
   */
  public int getImageWidth() {
    if (configurationUpdates > 0)
      return imageWidth;
    else
      return -1;
  }

  /**
   * Return the current configured image height of the OpenMV camera.
   * @return Image height feedback from camera.
   */
  public int getImageHeight() {
    if (configurationUpdates > 0)
      return imageHeight;
    else
      return -1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // See if we got a heartbeat and update data.
    if (readHeartbeat()) {

      // Only log hearbeat updates once in a while.
      if (loopCounter % 200 == 0) {
        System.out.format("OpenMV Heartbeat. Device ID: %d  Frame: %d  Heartbeat: %d\n", deviceId, frameCounter,
            lastHeartbeat);
      }

      missedHeartbeats = 0;
    } else {
      missedHeartbeats++;

      // If we miss too many complain about it.
      if (missedHeartbeats > 100) {
        // TODO: Do something more reasonable using Shuffleboard to tell which cameras are working.
        System.out.format("OpenMV Missing heartbeat. Device ID: %d\n", deviceId);
        missedHeartbeats = 0;
      }

    }

    // On H7's this does not do anything right now. Bug being worked by OpenMV
    // if ((loopCounter % 50 == 0)) {
    //   // System.out.println("SET MODE");
    //   setMode((byte)3);
    // }

    // See if we got a config message and update data, but only check once in a while.
    if ((loopCounter % 200 == 0) && readConfig()) {
      System.out.format("OpenMV Configuration Updated. Device ID: %d\n", deviceId);
    }

    // See if we got a camera status message and update data, but only check once in a while.
    if ((loopCounter % 200 == 0) && readCameraStatus())
    {
      System.out.format("OpenMV Camera Status Updated. Device ID: %d Image Width: %d Height: %d\n", deviceId, imageWidth, imageHeight);
    }

    // Read in the track slots that are available:
    for (int slot = 0; slot < getSimpleTargetTrackingSlots(); ++slot) {
      readSimpleTarget(slot);
    }

    SmartDashboard.putNumber("CAN/Track 0 CX", tracks.get(0).cx);
    SmartDashboard.putNumber("CAN/Track 1 CX", tracks.get(1).cx);
    SmartDashboard.putNumber("CAN/Trsck 2 CX", tracks.get(2).cx);
    SmartDashboard.putNumber("CAN/Track 3 CX", tracks.get(3).cx);
    SmartDashboard.putNumber("CAN/Frame Counter", frameCounter);
    
    loopCounter++;
  }

  boolean readSimpleTarget(int slot) {
    if (slot < 0 && slot >= tracks.size()) return false;

    if(read(apiIndex(2,slot), recvData) == true && recvData.length == 8) {
      var t = tracks.get(slot);
      // Packed 12 bits into 3 bytes:
      //    Byte 0:  High 8 of X
      //    Byte 1:  High 4 bits => Low 4 of X, Low 4 bits => High 4 of Y.
      //    Byte 2:  Low 8 of Y
      t.cx = (recvData.data[0] & 0xFF << 4) | ((recvData.data[1] & 0xF0) >> 4);
      t.cy = (recvData.data[1] & 0x0F) << 8 | recvData.data[2];
      t.vx = recvData.data[4] & 0xFF;
      t.vy = recvData.data[5] & 0xFF;
      t.type = recvData.data[5] & 0xFF;
      t.quality = recvData.data[6] & 0xFF;
      return true;
    } else {
      var t = tracks.get(slot);
      t.type = 0;
      t.quality = 0;
      return false;
    }
  }

  ArrayList<Track> getTracks() {
    return tracks;
  }
  
}
