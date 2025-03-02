// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagFinder extends SubsystemBase 
{
  public class VisionMeasurement
  {
    public Pose2d pose; // this is in field coordinates
    public double timeStamp;
    public int tagID;
    public double range;

    public VisionMeasurement(Pose2d pose, double timeStamp, int tagID, double range)
    {
      this.pose = pose;
      this.timeStamp = timeStamp;
      this.tagID = tagID;
      this.range = range;
    }
  }

  public PhotonCamera frontLeftCam = new PhotonCamera("FrontLeftCamera");
  public PhotonCamera frontRightCam = new PhotonCamera("FrontRightCamera");
  public PhotonCamera frontCenterCam = new PhotonCamera("FrontCenterCamera");

  //Camera height: 0.2159m, x and y: 0.264m
  public final Transform3d fLCamTransform3d = new Transform3d(new Translation3d(0.2925,0.2925, 0.216), new Rotation3d(0, 0, (Math.PI) / 4));
  public final Transform3d fRCamTransform3d = new Transform3d(new Translation3d(0.2925, -0.2925, 0.216), new Rotation3d(0, 0, -(Math.PI) / 4));
  public final Transform3d fCCamTransform3d = new Transform3d(new Translation3d(-0.2162302, 0, 0.79), new Rotation3d(0, Math.toRadians(15), 0));
  // public final Transform3d fCCamTransform3d = new Transform3d(new Translation3d(-0.2162302, 0, 0.79), new Rotation3d(0, 0, 0));
  public final Transform3d bLCamTransform3d = new Transform3d(new Translation3d(-0.2925, -0.2925, 0.216), new Rotation3d(0, 0, 3*(Math.PI) / 4));
  public final Transform3d bRCamTransform3d = new Transform3d(new Translation3d(0.2925, -0.2925, 0.216), new Rotation3d(0, 0, -3*(Math.PI) / 4));
  List<PhotonTrackedTarget> responseFL;
  List<PhotonTrackedTarget> responseFR;
  List<PhotonTrackedTarget> responseFC;
  List<PhotonTrackedTarget> responseBL;
  List<PhotonTrackedTarget> responseBR;

  double responseFLTimestamp;
  double responseFRTimestamp;
  double responseFCTimestamp;
  double responseBLTimestamp;
  double responseBRTimestamp;

  double ambiguityThreshold = 0.2; // TODO: verify this number


  public int wait_counter = 0;

  public List<PhotonTrackedTarget> getFLCurrentTagData() 
  {
    return responseFL;
  }

  public List<PhotonTrackedTarget> getFRCurrentTagData() 
  {
    return responseFR;
  }

  public List<PhotonTrackedTarget> getFCCurrentTagData()
  {
    return responseFC;
  }

  public List<PhotonTrackedTarget> getBLCurrentTagData() 
  {
    return responseBL;
  }

  public List<PhotonTrackedTarget> getBRCurrentTagData() 
  {
    return responseBR;
  }

  public void readTagData()
  {
    // TODO: use new method instead of getLatestResult()
    responseFL = frontLeftCam.getLatestResult().getTargets();
    responseFR = frontRightCam.getLatestResult().getTargets();
    responseFC = frontCenterCam.getLatestResult().getTargets();
    //responseBL = backLeftCam.getLatestResult().getTargets();
    //responseBR = backRightCam.getLatestResult().getTargets();
    responseFLTimestamp = Timer.getFPGATimestamp() - frontLeftCam.getLatestResult().metadata.getLatencyMillis() / 1000.0;
    responseFRTimestamp = Timer.getFPGATimestamp() - frontRightCam.getLatestResult().metadata.getLatencyMillis() / 1000.0;
    responseFCTimestamp = Timer.getFPGATimestamp() - frontCenterCam.getLatestResult().metadata.getLatencyMillis() / 1000.0;
    //responseBLTimestamp = Timer.getFPGATimestamp() - backLeftCam.getLatestResult().metadata.getLatencyMillis() / 1000.0;
    //responseBRTimestamp = Timer.getFPGATimestamp() - backRightCam.getLatestResult().metadata.getLatencyMillis() / 1000.0;
  }

  public ArrayList<VisionMeasurement> getMeasurements()
  {
    ArrayList<VisionMeasurement> measurements = new ArrayList<VisionMeasurement>();
    //return measurements;
    double range = 0;
    PhotonTrackedTarget targetFL = null;
    PhotonTrackedTarget targetFR = null;
    PhotonTrackedTarget targetFC = null;

    // front left camera
    //PhotonTrackedTarget targetFL = frontLeftCam.getLatestResult().getBestTarget();  // if you don't do hasTargets() first it throws a fit
    PhotonPipelineResult latestResultFL = frontLeftCam.getLatestResult();
    if (latestResultFL.hasTargets()) {
      targetFL = latestResultFL.getBestTarget();
    }
    
    if (targetFL != null && FieldMap.fieldMap.getTagPose(targetFL.getFiducialId()).isPresent())
    {
      Pose3d robotPoseFL = PhotonUtils.estimateFieldToRobotAprilTag(targetFL.getBestCameraToTarget(),
                                                                      FieldMap.fieldMap.getTagPose(targetFL.getFiducialId()).get(), 
                                                                      fLCamTransform3d.inverse());
      range = targetFL.bestCameraToTarget.getTranslation().getNorm();
      measurements.add(new VisionMeasurement(robotPoseFL.toPose2d(), responseFLTimestamp, targetFL.getFiducialId(), range));
    }
    
    // front right cameras
    //PhotonTrackedTarget targetFR = frontRightCam.getLatestResult()
    PhotonPipelineResult latestResultFR = frontRightCam.getLatestResult();
    if (latestResultFR.hasTargets()) {
      targetFR = latestResultFR.getBestTarget();
    }

    if (targetFR != null && FieldMap.fieldMap.getTagPose(targetFR.getFiducialId()).isPresent())
    {
      Pose3d robotPoseFR = PhotonUtils.estimateFieldToRobotAprilTag(targetFR.getBestCameraToTarget(),
                                                                      FieldMap.fieldMap.getTagPose(targetFR.getFiducialId()).get(), 
                                                                      fRCamTransform3d.inverse());
      range = targetFR.bestCameraToTarget.getTranslation().getNorm();
      measurements.add(new VisionMeasurement(robotPoseFR.toPose2d(), responseFRTimestamp, targetFR.getFiducialId(), range));
    }

    //PhotonTrackedTarget targetFC = frontCenterCam.getLatestResult().getBestTarget();
    PhotonPipelineResult latestResultFC = frontCenterCam.getLatestResult();
    if (latestResultFC.hasTargets()) {
      targetFC = latestResultFC.getBestTarget();
    }

    if (targetFC != null && FieldMap.fieldMap.getTagPose(targetFC.getFiducialId()).isPresent())
    {
      Pose3d robotPoseFC = PhotonUtils.estimateFieldToRobotAprilTag(targetFC.getBestCameraToTarget(),
                                                                      FieldMap.fieldMap.getTagPose(targetFC.getFiducialId()).get(), 
                                                                      fCCamTransform3d.inverse());
      range = targetFC.bestCameraToTarget.getTranslation().getNorm();
      measurements.add(new VisionMeasurement(robotPoseFC.toPose2d(), responseFCTimestamp, targetFC.getFiducialId(), range));
    }

    return measurements;
  }

  public ArrayList<VisionMeasurement> getAllGoodMeasurements() // TODO: untested
  {
    ArrayList<VisionMeasurement> measurements = new ArrayList<VisionMeasurement>();
    double range = 0;

    // front left camera
    ArrayList<PhotonTrackedTarget> targetsFL = new ArrayList<>(frontLeftCam.getLatestResult().getTargets());
    for (int i = 0; i < targetsFL.size(); i++)
    {
      if (targetsFL.get(i) != null && 
          FieldMap.fieldMap.getTagPose(targetsFL.get(i).getFiducialId()).isPresent() &&
          targetsFL.get(i).getPoseAmbiguity() < ambiguityThreshold && 
          targetsFL.get(i).getPoseAmbiguity() != -1) 
      {
        Pose3d robotPoseFL = PhotonUtils.estimateFieldToRobotAprilTag(targetsFL.get(i).getBestCameraToTarget(),
                                                            FieldMap.fieldMap.getTagPose(targetsFL.get(i).getFiducialId()).get(), 
                                                            fLCamTransform3d.inverse());
        range = targetsFL.get(i).bestCameraToTarget.getTranslation().getNorm();
        measurements.add(new VisionMeasurement(robotPoseFL.toPose2d(), responseFLTimestamp, targetsFL.get(i).getFiducialId(), range));
      }
    }
    
    
    // front right camera
    ArrayList<PhotonTrackedTarget> targetsFR = new ArrayList<>(frontRightCam.getLatestResult().getTargets());
    for (int i = 0; i < targetsFR.size(); i++)
    {
      if (targetsFR.get(i) != null && 
          FieldMap.fieldMap.getTagPose(targetsFR.get(i).getFiducialId()).isPresent() &&
          targetsFR.get(i).getPoseAmbiguity() < ambiguityThreshold && 
          targetsFR.get(i).getPoseAmbiguity() != -1) 
      {
        Pose3d robotPoseFR = PhotonUtils.estimateFieldToRobotAprilTag(targetsFR.get(i).getBestCameraToTarget(),
                                                            FieldMap.fieldMap.getTagPose(targetsFR.get(i).getFiducialId()).get(), 
                                                            fRCamTransform3d.inverse());
        range = targetsFR.get(i).bestCameraToTarget.getTranslation().getNorm();
        measurements.add(new VisionMeasurement(robotPoseFR.toPose2d(), responseFRTimestamp, targetsFR.get(i).getFiducialId(), range));
      }
    }

    ArrayList<PhotonTrackedTarget> targetsFC = new ArrayList<>(frontCenterCam.getLatestResult().getTargets());
    for (int i = 0; i < targetsFC.size(); i++)
    {
      if (targetsFC.get(i) != null && 
          FieldMap.fieldMap.getTagPose(targetsFC.get(i).getFiducialId()).isPresent() &&
          targetsFC.get(i).getPoseAmbiguity() < ambiguityThreshold && 
          targetsFC.get(i).getPoseAmbiguity() != -1) 
      {
        Pose3d robotPoseFC = PhotonUtils.estimateFieldToRobotAprilTag(targetsFC.get(i).getBestCameraToTarget(),
                                                            FieldMap.fieldMap.getTagPose(targetsFC.get(i).getFiducialId()).get(), 
                                                            fCCamTransform3d.inverse());
        range = targetsFC.get(i).bestCameraToTarget.getTranslation().getNorm();
        measurements.add(new VisionMeasurement(robotPoseFC.toPose2d(), responseFCTimestamp, targetsFC.get(i).getFiducialId(), range));
      }
    }

    return measurements;
  }

  public Transform3d getRobotToFRCam() {
    return fRCamTransform3d;
  }

  public Transform3d getRobotToFLCam() {
    return fCCamTransform3d;
  }

  public Transform3d getRobotToFCCam() {
    return fCCamTransform3d;
  }

  @Override
  public void periodic() 
  { 
    readTagData();  

    if(responseFL.size() > 0) 
    {
      SmartDashboard.putNumber("FL ID", responseFL.get(0).getFiducialId());
    }
    else 
    {
      SmartDashboard.putNumber("FL ID", -1);
    }
    if(responseFR.size() > 0) 
    {
      SmartDashboard.putNumber("FR ID", responseFR.get(0).getFiducialId());
    }
    else 
    {
      SmartDashboard.putNumber("FR ID", -1);
    }
    SmartDashboard.putNumber("Total Tags Seen", responseFL.size() + responseFR.size());
    if(getMeasurements().size() > 0) 
    {
      //SmartDashboard.putNumber("FL Measurement X", getMeasurements().get(0).pose.getX());
      //SmartDashboard.putNumber("FL Measurement Y", getMeasurements().get(0).pose.getY());
    }
  }
}