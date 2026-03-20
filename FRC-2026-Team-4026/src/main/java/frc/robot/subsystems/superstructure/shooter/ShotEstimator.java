package frc.robot.subsystems.superstructure.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
    public static ShotEstimator instance;
    
    public ShotEstimator(){
        this.drive = RobotContainer.getDrive();
        targetVelocities = new InterpolatingDoubleTreeMap();
        targetVelocities.put(2.057, 36.5);
        targetVelocities.put(2.62, 38.5);
        targetVelocities.put(2.87, 40.5);
        targetVelocities.put(3.386, 44.5);
        targetVelocities.put(4.21, 53.5);
        targetVelocities.put(4.97, 56.5);
    }

    @Override
    public void periodic(){
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            robotDistance = drive.getPose().getTranslation().getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        } else{
            robotDistance = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply((FieldConstants.Hub.topCenterPoint.toTranslation2d())));        
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


}
