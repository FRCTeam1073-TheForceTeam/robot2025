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
import edu.wpi.first.math.geometry.Transform2d;
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
    public Transform2d relativePose; // this is in robot coordinates.
    public double timeStamp;
    public int tagID;
    public double range;

    public VisionMeasurement(Pose2d pose, Transform2d relativePose, double timeStamp, int tagID, double range)
    {
      this.pose = pose;
      this.relativePose = relativePose;
      this.timeStamp = timeStamp;
      this.tagID = tagID;
      this.range = range;
    }
  }

  public PhotonCamera frontLeftCam = new PhotonCamera("FrontLeftCamera");  
  public PhotonCamera frontCenterCam = new PhotonCamera("FrontCenterCamera");
  public PhotonCamera frontRightCam = new PhotonCamera("FrontRightCamera");
  public PhotonCamera rearCam = new PhotonCamera("RearCamera");

  //Camera height: 0.2159m, x and y: 0.264m
  public final Transform3d fLCamTransform3d = new Transform3d(new Translation3d(0.2925,0.2925, 0.216), new Rotation3d(0, 0, (Math.PI) / 4));
  public final Transform3d fCCamTransform3d = new Transform3d(new Translation3d(-0.2162302, -0.127, 0.673), new Rotation3d(0, Math.toRadians(15), 0));
  public final Transform3d fRCamTransform3d = new Transform3d(new Translation3d(0.2925, -0.2925, 0.216), new Rotation3d(0, 0, -(Math.PI) / 4));
  public final Transform3d rCamTransform3d = new Transform3d(new Translation3d(-0.1969, -0.127, 0.673), new Rotation3d(0, Math.toRadians(-15), Math.PI)); //TODO: check this number

  double ambiguityThreshold = 0.28; // TODO: verify this number

  public ArrayList<VisionMeasurement> getAllMeasurements() {
    ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();
    visionMeasurements.addAll(getCamMeasurements(frontLeftCam, fLCamTransform3d));
    visionMeasurements.addAll(getCamMeasurements(frontCenterCam, fCCamTransform3d));
    visionMeasurements.addAll(getCamMeasurements(frontRightCam, fRCamTransform3d));
    visionMeasurements.addAll(getCamMeasurements(rearCam, rCamTransform3d));

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
              var best = target.getBestCameraToTarget();
              Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(best,
                                                  FieldMap.fieldMap.getTagPose(target.getFiducialId()).get(), 
                                                  camTransform3d.inverse());
              range = target.bestCameraToTarget.getTranslation().getNorm();
              var relativePose = toTransform2d(camTransform3d.plus(best));
              measurements.add(new VisionMeasurement(robotPose.toPose2d(), relativePose, responseTimestamp, target.getFiducialId(), range));
            }
          }
        }
      }
    }
    return measurements;
  }

  public Transform2d toTransform2d(Transform3d t3d) {
    return new Transform2d(t3d.getX(), t3d.getY(), t3d.getRotation().toRotation2d());
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

  public Transform3d getRobotToRCam() {
    return rCamTransform3d;
  }

  @Override
  public void periodic() 
  {   
  }

}