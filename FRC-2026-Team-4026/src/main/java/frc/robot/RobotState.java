package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;

public class RobotState {
    // add params for PDH object
    private double totalCurrentDraw;
    private int currentLimit = 180;
    private double busVoltage;
    public static boolean CURRENT_LIMITS_EXCEEDED;
    public static boolean BATTERY_BROWNOUT_PROTECTION;
    private double brownoutProtectionVoltage = 9;
    private PowerDistribution PDH = new PowerDistribution();

    private Drive drive;
    private Pose2d robotPose = drive.getPose();
    private InterpolatingDoubleTreeMap targetAims = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap targetVelocities = new InterpolatingDoubleTreeMap();
    private Double robotDistance = Math.hypot(robotPose.getX() - FieldConstants.Hub.topCenterPoint.getX(), robotPose.getY() - FieldConstants.Hub.topCenterPoint.getY());
    private SwerveDriveSimulation driveSimulation;
    private Double speedOffset = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    
public RobotState(){
    targetAims.put(0.0, 0.0);
    targetVelocities.put(0.0, 20.0);
    totalCurrentDraw = PDH.getTotalCurrent();
    busVoltage = PDH.getVoltage();
}

    public void periodic(){
        busVoltage = PDH.getVoltage();
        totalCurrentDraw = PDH.getTotalCurrent();

        if (totalCurrentDraw > currentLimit){

            CURRENT_LIMITS_EXCEEDED = true;

        }
        if (busVoltage < brownoutProtectionVoltage){
            
            BATTERY_BROWNOUT_PROTECTION = true;
        }

        
    }
    public double getTargetAim(){
        return targetAims.get(robotDistance);
    }

    public double getTargetVelocity(){
        return targetVelocities.get(robotDistance);
    }

    public double getSpeedOffsetShooter() {
        return speedOffset;
    }


    public double getTurretRotation() {
        return Math.atan((robotPose.getY() - FieldConstants.Hub.topCenterPoint.getY())/(robotPose.getX() - FieldConstants.Hub.topCenterPoint.getX()));
    }
    public Translation2d getDrivePose(){
        return new Translation2d(drive.getPose().getX(),drive.getPose().getY());
    }

    public ChassisSpeeds getChassisSpeed(){
        return drive.getChassisSpeeds();
    }

    public Rotation2d getDriveRotatoin(){
        return drive.getRotation();
    }
}
