package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIOTalonFX;

public class RobotState {
    // add params for PDH object
    private double totalCurrentDraw;
    private int currentLimit = 180;
    private double busVoltage;
    public static boolean CURRENT_LIMITS_EXCEEDED;
    public static boolean BATTERY_BROWNOUT_PROTECTION;
    private PowerDistribution PDH = new PowerDistribution();
    private double brownoutProtectionVoltage = 1.432*Math.log10(PDH.getVoltage()-7);

    private Drive drive;
    private Intake intake;
    private Pose2d robotPose;
    private InterpolatingDoubleTreeMap targetAims;
    private InterpolatingDoubleTreeMap targetVelocities;
    private Double robotDistance;
    private Double speedOffset;
      private InterpolatingDoubleTreeMap voltageToVelocity = new InterpolatingDoubleTreeMap();
    private Timer timer;
    private boolean hasIntaked;
    private RobotContainer robotContainer;
public RobotState(Drive drive){
    this.drive = drive;
    robotContainer = RobotContainer.getInstance();
    robotPose = drive.getPose();
    targetAims = new InterpolatingDoubleTreeMap();
    targetVelocities = new InterpolatingDoubleTreeMap();
    robotDistance = Math.hypot(robotPose.getX() - FieldConstants.Hub.topCenterPoint.getX(), robotPose.getY() - FieldConstants.Hub.topCenterPoint.getY());
    speedOffset = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    targetAims.put(0.1, 0.0);
    targetVelocities.put(1.0, 30.0);
    targetAims.put(1.0, 0.1);
    targetVelocities.put(2.0, 40.0);
      voltageToVelocity.put(0.0, 0.0);
    voltageToVelocity.put(6.0, 40.0);
    voltageToVelocity.put(8.0, 64.0);
    voltageToVelocity.put(12.0, 85.0);
    totalCurrentDraw = PDH.getTotalCurrent();
    busVoltage = PDH.getVoltage();
    timer = new Timer();
    intake = new Intake(new IntakeIOTalonFX());
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

        if (DriverStation.isAutonomous() && intake.isDeployed()){
            if (timer.isRunning() == false){
                hasIntaked = false;
                timer.reset();
                timer.start();
            }
            else{
                if( intake.isActuallyIntaking()){
                    hasIntaked = true;
                }
            }
            if (timer.hasElapsed(1)){
                if (hasIntaked == false){
                    robotContainer.changePathOverideFeedbackCommand();
                }
                timer.stop();
            }
        }
    }

    public double getVoltageToVelocity(double voltage){
    return voltageToVelocity.get(voltage);
}

public double getBrownoutVoltage() {
    return brownoutProtectionVoltage;
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
