package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.ejml.equation.MatrixConstructor;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;

public class RobotState extends SubsystemBase
{
    private static RobotState instance;
    // add params for PDH object
    private double totalCurrentDraw;
    private int currentLimit = 180;
    private double busVoltage;
    public static boolean CURRENT_LIMITS_EXCEEDED;
    public static boolean BATTERY_BROWNOUT_PROTECTION;
    //private PowerDistribution PDH = new PowerDistribution();
   // private double brownoutProtectionVoltage = 1.432*Math.log10(PDH.getVoltage()-7);

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
    
public RobotState(Drive drive){
    this.drive = drive;
    instance = this;
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
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
   // totalCurrentDraw = PDH.getTotalCurrent();
    //busVoltage = PDH.getVoltage();
}


// public void resetPose(Pose2d newPose){
//     gyroOffset = newPose.getRotation().minus(drive.getRotation().minus(gyroOffset));
//     odemetryPose = newPose;
//     estimatedPose = newPose;
//     poseBuffer.clear();
// }

// public void addOdometryPose(OdometryObservation observation){
//     Twist2d twist = kinematics.toTwist2d(modulePositions, observation.modulePositions);
//     modulePositions = observation.modulePositions;
//     Pose2d lastOdometryPose = odemetryPose;
//     odemetryPose = odemetryPose.exp(twist);

//     observation.gyroRotation.ifPresent(
//         gyroRotation -> {
//             Rotation2d angle = gyroRotation.plus(gyroOffset);
//             odemetryPose = new Pose2d(odemetryPose.getTranslation(), angle);
//         }
//     );

//     poseBuffer.addSample(observation.timestamp, odemetryPose);

//     Twist2d finalTwist = lastOdometryPose.log(odemetryPose);
//     estimatedPose = estimatedPose.exp(finalTwist);
// }

// public void addVisionPose(){
//     try{
//         if(poseBuffer.getInternalBuffer().lastKey() - poseBufferTime > visionTimestamp){
//             return;
//         }
//     }
//     catch(NoSuchElementException e){
//         return;}

//     var sample = poseBuffer.getSample(visionTimestamp);
//     if(sample.isEmpty()){
//         return;
//     }

//     var sampleToOdometryTransform = new Transform2d(sample.get(), odemetryPose);
//     var odometryToSampleTransform = new Transform2d(odemetryPose, sample.get());

//     Pose2d estimateAtTimestamp = estimatedPose.transformBy(odometryToSampleTransform);

//     Matrix<N3, N3> kalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
//     for(int row = 0; row < 3; row++){
//         double stdDev = visionMeasurementStdDevs.get(row, 0);
//         if(stdDev == 0){
//             kalmanGain.set(row, row, 0);
//         }
//         else{
//             kalmanGain.set(row, row, stdDev/(stdDev + Math.sqrt(stdDev*visionMeasurementStdDevs.get(row,0))));
//         }
//     }

//     Transform2d poseTransform = new Transform2d(estimateAtTimestamp, visionPose);

//     var kalmanTransform = kalmanGain.times(VecBuilder.fill(poseTransform.getX(), poseTransform.getY(), poseTransform.getRotation().getRadians()));

//     Transform2d scaledTransform = new Transform2d(kalmanTransform.get(0, 0), kalmanTransform.get(1, 0), new Rotation2d(kalmanTransform.get(2, 0)));

//     estimatedPose = estimateAtTimestamp.plus(scaledTransform).plus(sampleToOdometryTransform);

// }
@Override
    public void periodic(){
       // busVoltage = PDH.getVoltage();
       // totalCurrentDraw = PDH.getTotalCurrent();

        // if (totalCurrentDraw > currentLimit){

        //     CURRENT_LIMITS_EXCEEDED = true;

        // }
        // if (busVoltage < brownoutProtectionVoltage){
            
        //     BATTERY_BROWNOUT_PROTECTION = true;
        // }
        robotPose = drive.getPose();
        robotDistance = Math.hypot(robotPose.getX() - FieldConstants.Hub.topCenterPoint.getX(), robotPose.getY() - FieldConstants.Hub.topCenterPoint.getY());
        
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
        return estimatedPose.getTranslation();
    }

    public ChassisSpeeds getChassisSpeed(){
        return drive.getChassisSpeeds();
    }

    public Rotation2d getDriveRotatoin(){
        return drive.getRotation();
    }

//     @Override
//  public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
//     visionPose = visionRobotPoseMeters;
//     visionTimestamp = timestampSeconds;
//         this.visionMeasurementStdDevs = visionMeasurementStdDevs;
//     }

//     public record OdometryObservation(double timestamp, SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroRotation){ 

//     }

    public static RobotState getInstance(){
        return instance;
    }
}
