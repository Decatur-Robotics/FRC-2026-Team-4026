package frc.robot.subsystems.superstructure.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ShotEstimator extends SubsystemBase{
    private Drive drive;
    private InterpolatingDoubleTreeMap targetVelocities;
    public double robotDistance;
    public double passingDistance;
    public static ShotEstimator instance;
    
    public ShotEstimator(){
        this.drive = RobotContainer.getDrive();
        targetVelocities = new InterpolatingDoubleTreeMap();
        targetVelocities.put(2.057, 37.0);
        targetVelocities.put(2.62, 39.0);
        targetVelocities.put(2.87, 41.0);
        targetVelocities.put(3.386, 45.0);
        targetVelocities.put(4.21, 54.0);
        targetVelocities.put(4.97, 57.0);
    }

    @Override
    public void periodic(){
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            robotDistance = drive.getPose().getTranslation().getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d());
            passingDistance = 5 + drive.getPose().getTranslation().getDistance(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(),drive.getPose().getX()));
        } else{
            robotDistance = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply((FieldConstants.Hub.topCenterPoint.toTranslation2d())));
            passingDistance = 5 + drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(),drive.getPose().getX())));        
        }

        
        Logger.recordOutput("ShotEstimator/Distance", robotDistance);
        Logger.recordOutput("ShotEstimator/Velocity", getTargetVelocity().get());
    }

     public static ShotEstimator getInstance(){
         if(instance == null){
            instance = new ShotEstimator();
    }
     return instance;
     }

    public Supplier<Double> getTargetVelocity(){
         return () ->  targetVelocities.get(robotDistance);
        // return () -> 10.07*robotDistance+18.98;
    }

    public Supplier<Double> getPassingVelocity(){
        return () -> targetVelocities.get(passingDistance);
    }


}
