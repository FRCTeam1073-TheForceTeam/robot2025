// MAP: loads the map

package frc.robot.subsystems;


import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FieldMap
{
    // TODO: AprilTagFields has two flavors of reefscape map: andymark and welded. which one do you need? who knows!
    // apparently for the 2/15 event it's welded. after that all bets are off.
    public static final AprilTagFieldLayout fieldMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public int getBestAprilTagID(Pose2d robotPose) 
    {
        double shortestDistance = 998;

        List<AprilTag> aprilTags = fieldMap.getTags();
        int bestID = -1;

        for(AprilTag tag : aprilTags) 
        {
            if (findDistance(robotPose, tag.ID) < shortestDistance && tag.ID != 4 && tag.ID != 5 && tag.ID != 14 && tag.ID != 15) 
            {
                shortestDistance = findDistance(robotPose, bestID);
                bestID = tag.ID;
            }
        }
        return bestID;
    }

    public Pose2d getTagRelativePose(int tagID, int slot, Transform2d offset)
    {
        Pose2d tagPose = fieldMap.getTagPose(tagID).get().toPose2d();
        double yOffset = 0.5;

        if(slot == -1) // left
        {
            tagPose = tagPose.plus(new Transform2d(0, -yOffset, new Rotation2d()));
        }
        else if(slot == 0) // center
        {
            
        }
        else if(slot == 1) // right
        {
            tagPose = tagPose.plus(new Transform2d(0, yOffset, new Rotation2d()));
        }

        tagPose = tagPose.plus(offset);

        SmartDashboard.putNumber("FieldMap/TargetTagID", tagID);
        SmartDashboard.putNumber("FieldMap/TargetPoseX", tagPose.getX());
        SmartDashboard.putNumber("FieldMap/TargetPoseY", tagPose.getY());
        SmartDashboard.putNumber("FieldMap/TargetPoseTheta", tagPose.getRotation().getRadians());

        return tagPose;
    }

    public double findDistance(Pose2d robot2DPose, int tagID) {

        Optional<Pose3d> tag3dPose = FieldMap.fieldMap.getTagPose(tagID);

        if(!tag3dPose.isPresent()) {
            //we don't have a value to work with, bail
            return 999;
        }

        double tagX = tag3dPose.get().getX();
        double tagY = tag3dPose.get().getY();
        double distance = Math.sqrt(Math.pow((tagX - robot2DPose.getX()), 2) + Math.pow((tagY - robot2DPose.getY()), 2));
        return distance;
    }
}
