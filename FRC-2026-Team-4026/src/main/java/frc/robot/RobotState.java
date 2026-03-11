package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.ejml.equation.MatrixConstructor;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeometryUtil;


public class RobotState extends SubsystemBase
{
    // add params for PDH object
    private double totalCurrentDraw;
    private int currentLimit = 180;
    private double busVoltage;
    public static boolean CURRENT_LIMITS_EXCEEDED;
    public static boolean BATTERY_BROWNOUT_PROTECTION;
   private PowerDistribution PDH = new PowerDistribution();
//    private double brownoutProtectionVoltage = 1.432*Math.log10(PDH.getVoltage()-7);

    private static final double poseBufferTime = 2.0; // seconds

    @AutoLogOutput private Pose2d odemetryPose = Pose2d.kZero;
    @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
     @AutoLogOutput private Pose2d visionPose = Pose2d.kZero;

      private Rotation2d gyroOffset = Rotation2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferTime);
    private Matrix<N3, N1> visionMeasurementStdDevs;

    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private double visionTimestamp;

    private Drive drive;
    private Pose2d robotPose;
    private InterpolatingDoubleTreeMap targetAims;
    private InterpolatingDoubleTreeMap targetVelocities;
    private Double robotDistance;
    private Double speedOffset;
    private InterpolatingDoubleTreeMap voltageToVelocity = new InterpolatingDoubleTreeMap();
    
    private Translation2d hubToRobot;
    private static RobotState instance;
public RobotState(Drive drive){
    this.drive = drive;
    instance = this;
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    robotPose = drive.getPose();
    targetAims = new InterpolatingDoubleTreeMap();
    targetVelocities = new InterpolatingDoubleTreeMap();
    speedOffset = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    hubToRobot = new Translation2d(robotPose.getX() - FieldConstants.Hub.topCenterPoint.getX(), robotPose.getY() - FieldConstants.Hub.topCenterPoint.getY());
}


public void resetPose(Pose2d newPose){
    gyroOffset = newPose.getRotation().minus(drive.getRotation().minus(gyroOffset));
    odemetryPose = newPose;
    estimatedPose = newPose;
    poseBuffer.clear();
}

@Override
    public void periodic(){
    //    busVoltage = PDH.getVoltage();
    //    totalCurrentDraw = PDH.getTotalCurrent();

    //     if (totalCurrentDraw > currentLimit){

    //         CURRENT_LIMITS_EXCEEDED = true;

    //     }
    //     if (busVoltage < brownoutProtectionVoltage){
            
    //         BATTERY_BROWNOUT_PROTECTION = true;
    //     }
    }

    public double getVoltageToVelocity(double voltage){
        return voltageToVelocity.get(voltage);
    }

// public double getBrownoutVoltage() {
//     return brownoutProtectionVoltage;
// }
    public double getTargetAim(){
        return targetAims.get(robotDistance);
    }

    public double getSpeedOffsetShooter() {
        return speedOffset;
    }

    public double getTurretRotation() {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            return Math.atan((drive.getPose().getY() - FieldConstants.Hub.topCenterPoint.getY())/(drive.getPose().getX() - FieldConstants.Hub.topCenterPoint.getX()));
        } else {
            return Math.atan((drive.getPose().getY() - AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.toTranslation2d().getY()))/(drive.getPose().getX() - AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX()))) + Math.PI;
        }
    }
    public Translation2d getDrivePose(){
        return estimatedPose.getTranslation();
    }

    public ChassisSpeeds getChassisSpeed(){
        return drive.getChassisSpeeds();
    }

    public Rotation2d getDriveRotatoin(){
        return drive.getRotation();
    }

     
    public static RobotState getInstance() {
         return instance;
    }

    public static Pose2d getEstimatedRobotPose(){
        return instance.drive.getPose();
    }

    public double getChassisVelocity(){
        return GeometryUtil.getChassisTranslationSpeeds(drive.getChassisSpeeds());

    }

    public ChassisSpeeds getFieldVelocity(){
       return ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

}
    public Translation2d getHubLocalizedRobotPose(){
        return hubToRobot;
    }


}