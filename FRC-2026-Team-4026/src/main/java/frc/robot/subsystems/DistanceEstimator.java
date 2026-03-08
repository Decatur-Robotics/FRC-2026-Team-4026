package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.template.Vision;
import frc.robot.util.AllianceFlipUtil;

public class DistanceEstimator extends SubsystemBase implements Vision.VisionConsumer {
    private double robotDistance;
    private Pose2d robotPose;

    public DistanceEstimator(){
        robotDistance = 2.0;
        robotPose = new Pose2d(3,3,new Rotation2d());
    }

    @Override
    public void periodic(){
        if(robotPose != null){
        // if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            // robotDistance = robotPose.getTranslation().getDistance((FieldConstants.Hub.topCenterPoint.toTranslation2d()));
        // } else{
            robotDistance = robotPose.getTranslation().getDistance(AllianceFlipUtil.apply((FieldConstants.Hub.topCenterPoint.toTranslation2d())));
        // }
    }
    Logger.recordOutput("DistanceEstimator/Distance", robotDistance);
    }

    public double getDistance(){
        return robotDistance;
    }

         @Override
     public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
         robotPose = visionRobotPoseMeters;
     }
}
