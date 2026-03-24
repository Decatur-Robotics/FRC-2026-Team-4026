package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeometryUtil;


public class RobotState extends SubsystemBase
{


    public static boolean CURRENT_LIMITS_EXCEEDED;
    public static boolean BATTERY_BROWNOUT_PROTECTION;

//    private double brownoutProtectionVoltage = 1.432*Math.log10(PDH.getVoltage()-7);

    private static final double poseBufferTime = 2.0; // seconds

    @AutoLogOutput private Pose2d odemetryPose = Pose2d.kZero;
    @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
     @AutoLogOutput private Pose2d visionPose = Pose2d.kZero;

      private Rotation2d gyroOffset = Rotation2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferTime);



    private Drive drive;
    private Pose2d robotPose;
    private InterpolatingDoubleTreeMap targetAims;

    private Double robotDistance;
    private Double speedOffset;
    private InterpolatingDoubleTreeMap voltageToVelocity = new InterpolatingDoubleTreeMap();
    
    private Translation2d hubToRobot;
    private static RobotState instance;
public RobotState(Drive drive){
    this.drive = drive;
    instance = this;

    robotPose = drive.getPose();
    targetAims = new InterpolatingDoubleTreeMap();

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