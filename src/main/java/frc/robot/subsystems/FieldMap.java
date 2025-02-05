// MAP: loads the map

package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;


public class FieldMap
{
    public static final AprilTagFieldLayout fieldMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); 

    
    
    private Localizer localizer;
    private Pose2d robot2DPose = localizer.getPose();
    private ArrayList<Double> distances = new ArrayList<Double>();
   
    private List<AprilTag> aprilTags = fieldMap.getTags();
    
    public FieldMap()
    {

    }

    public int getBestAprilTag(Pose2d robot2DPose, List<AprilTag> aprilTags){
        int tagID = -1;

        for(AprilTag tag : aprilTags) {
            distances.add(findDistance(robot2DPose, tag.ID));
        }

        
        //return aprilTags.sort(lambda=AprilTag.findDistance(robot2DPose, tagID));

        return tagID;
    }

    public double findDistance(Pose2d robot2DPose, int tagID){

        Optional<Pose3d> tag3dPose = fieldMap.getTagPose(tagID);

        if (!tag3dPose.isPresent())
        {
            //we don't have a value to work with, bail
            return -1.0;
        }

        double tagX = tag3dPose.get().getX();
        double tagY = tag3dPose.get().getY();

        double distance = Math.sqrt(Math.pow((tagX - robot2DPose.getX()), 2) + Math.pow((tagY - robot2DPose.getY()), 2));
        return distance;
    }

}
