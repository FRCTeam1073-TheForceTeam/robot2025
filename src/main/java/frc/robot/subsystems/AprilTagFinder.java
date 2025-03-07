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
  public PhotonCamera frontCenterCam = new PhotonCamera("FrontCenterCamera");
  public PhotonCamera frontRightCam = new PhotonCamera("FrontRightCamera");

  //Camera height: 0.2159m, x and y: 0.264m
  public final Transform3d fLCamTransform3d = new Transform3d(new Translation3d(0.2925,0.2925, 0.216), new Rotation3d(0, 0, (Math.PI) / 4));
  public final Transform3d fCCamTransform3d = new Transform3d(new Translation3d(-0.2162302, -0.127, 0.673), new Rotation3d(0, Math.toRadians(15), 0));
  public final Transform3d fRCamTransform3d = new Transform3d(new Translation3d(0.2925, -0.2925, 0.216), new Rotation3d(0, 0, -(Math.PI) / 4));
  


  double ambiguityThreshold = 0.28; // TODO: verify this number


  public ArrayList<VisionMeasurement> getAllMeasurements() {
    ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();
    visionMeasurements.addAll(getCamMeasurements(frontLeftCam, fLCamTransform3d));
    visionMeasurements.addAll(getCamMeasurements(frontCenterCam, fCCamTransform3d));
    visionMeasurements.addAll(getCamMeasurements(frontRightCam, fRCamTransform3d));

    return visionMeasurements;
  }

  public ArrayList<PhotonTrackedTarget> getCamTargets(PhotonCamera camera) {

    ArrayList<PhotonPipelineResult> results = new ArrayList<>(camera.getAllUnreadResults());
    ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();

    for (PhotonPipelineResult result : results){
        if (result.hasTargets()){
          targets.addAll(result.getTargets());
        }
    }
    return targets;
  }

  public ArrayList<VisionMeasurement> getCamMeasurements(PhotonCamera camera, Transform3d camTransform3d) {
    ArrayList<VisionMeasurement> measurements = new ArrayList<VisionMeasurement>();
    ArrayList<PhotonPipelineResult> results = new ArrayList<>(camera.getAllUnreadResults());
    ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();

    for (PhotonPipelineResult result : results){
        if (result.hasTargets()){
          targets.addAll(result.getTargets());
        
        var responseTimestamp = Timer.getFPGATimestamp() - result.metadata.getLatencyMillis() / 1000.0;
        double range = 0;

        for (PhotonTrackedTarget target : targets) {
          if (FieldMap.fieldMap.getTagPose(target.getFiducialId()).isPresent()){
            if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() < ambiguityThreshold){
              Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                                                  FieldMap.fieldMap.getTagPose(target.getFiducialId()).get(), 
                                                  camTransform3d.inverse());
              range = target.bestCameraToTarget.getTranslation().getNorm();
              measurements.add(new VisionMeasurement(robotPose.toPose2d(), responseTimestamp, target.getFiducialId(), range));
            }
          }
        }
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
  }

}