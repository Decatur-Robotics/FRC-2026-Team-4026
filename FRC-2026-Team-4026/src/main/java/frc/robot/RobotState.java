package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.constants.FieldConstants;

public class RobotState {
    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d());
    private InterpolatingDoubleTreeMap targetAims = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap targetVelocities = new InterpolatingDoubleTreeMap();
    private Double robotDistance = Math.hypot(robotPose.getX() - FieldConstants.HUB_POSE_BLUE.getX(), robotPose.getY() - FieldConstants.HUB_POSE_BLUE.getY());
    
public RobotState(){
    targetAims.put(0.0, 0.0);
    targetVelocities.put(0.0, 20.0);
}
    public double getTargetAim(){
        return targetAims.get(robotDistance);
    }

    public double getTargetVelocity(){
        return targetVelocities.get(robotDistance);
    }


    public double getTurretRotation() {
        return Math.atan((robotPose.getY() - FieldConstants.HUB_POSE_BLUE.getY())/(robotPose.getX() - FieldConstants.HUB_POSE_BLUE.getX()));
    }
}
