package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;


public class DrivePathing extends Command{
    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    private static final LoggedTunableNumber DrivekP = new LoggedTunableNumber("DrivePathing/kP", 1.0);

    private static final LoggedTunableNumber DrivekD = new LoggedTunableNumber("DrivePathing/kD", 0.0);
    private static final LoggedTunableNumber ThetakP = new LoggedTunableNumber("DrivePathing/Theta/kP", 1.0);
    private static final LoggedTunableNumber ThetakD = new LoggedTunableNumber("DrivePathing/Theta/kD", 0.0);

    private static final LoggedTunableNumber driveMaxSpeed = new LoggedTunableNumber("DrivePathing/maxSpeed", 1.0);
    private static final LoggedTunableNumber driveMinSpeed = new LoggedTunableNumber("DrivePathing/minSpeed", 0.01);
    private static final LoggedTunableNumber driveMaxAngularSpeed = new LoggedTunableNumber("DrivePathing/maxAngularSpeed", 1.0);
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber("DrivePathing/maxAcceleration", 1.0);
    private static final LoggedTunableNumber driveMaxAngularAcceleration = new LoggedTunableNumber("DrivePathing/maxAngularAcceleration", 1.0);

    private static final LoggedTunableNumber driveFFMaxRadius = new LoggedTunableNumber("DrivePathing/FFMaxRadius", 1.0);
    private static final LoggedTunableNumber driveFFMinRadius = new LoggedTunableNumber("DrivePathing/FFMinRadius", 0.1);
    private static final LoggedTunableNumber thetaFFMinError = new LoggedTunableNumber("DrivePathing/ThetaFFMinRadius", 0.1);
    private static final LoggedTunableNumber thetaFFMaxError = new LoggedTunableNumber("DrivePathing/FFMaxRadius", 1.0);

    private static final LoggedTunableNumber driveTolerance = new LoggedTunableNumber("DrivePathing/tolerance", 0.01);

    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;

    @AutoLogOutput private boolean runningPathing = false;

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastTargetRotation = Rotation2d.kZero;

    private double lastTime = 0.0;

    private TrapezoidProfile driveProfile;
    private final PIDController driveController = new PIDController(0,0,0,0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(driveMaxAngularSpeed.get(), driveMaxAngularAcceleration.get()));
    public DrivePathing(Drive drive, Supplier<Pose2d> targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        initialize();
        execute();

    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        Pose2d target = targetPose.get();
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        Translation2d linearFieldSpeeds = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(driveMaxSpeed.get(), driveMaxAcceleration.get()));
        
        driveController.reset();
       thetaController.reset(currentPose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);

        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointVelocity = linearFieldSpeeds;
        lastTargetRotation = currentPose.getRotation();
        lastTime = Timer.getTimestamp();
        
    }

    @Override
    public void execute(){
        runningPathing = true;
        // if(driveTolerance.hasChanged(hashCode())){

        // }

        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(driveMaxSpeed.get(), driveMaxAcceleration.get()));
        thetaController.setConstraints(new TrapezoidProfile.Constraints(driveMaxAngularSpeed.get(), driveMaxAngularAcceleration.get()));

        Pose2d currentPose = drive.getPose();
        Pose2d target = targetPose.get();

        Pose2d poseError = currentPose.relativeTo(target);
        driveErrorAbs = poseError.getTranslation().getNorm();
        thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

        double driveFFScalar = MathUtil.clamp((driveErrorAbs - driveFFMinRadius.get()) 
        / (driveFFMaxRadius.get() - driveFFMinRadius.get()), 0.0, 1.0);

        double thetaFFScalar = MathUtil.clamp((thetaErrorAbs - thetaFFMinError.get())
        / (thetaFFMaxError.get() - thetaFFMinError.get()), 0.0, 1.0);

        var direction = targetPose.get().getTranslation().minus(lastSetpointTranslation).toVector();
        double targetVelocity = direction.norm() <= driveMinSpeed.get() 
        ? lastSetpointVelocity.getNorm() : lastSetpointVelocity.toVector().dot(direction)/direction.norm();

        State setpoint = driveProfile.calculate(0.02, new State(direction.norm(), targetVelocity), new State(0,0));

        double driveVelocityScalar = driveController.calculate(driveErrorAbs, setpoint.position)+ driveFFScalar * setpoint.velocity;

        
        if(driveErrorAbs <= driveTolerance.get()){
            driveVelocityScalar = 0.0;
        }

        Rotation2d targetDeltaAngle = currentPose.getTranslation().minus(targetPose.get().getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetDeltaAngle);
        lastSetpointTranslation = new Pose2d(target.getTranslation(), targetDeltaAngle).transformBy(GeometryUtil.toTransform2d(new Translation2d(setpoint.position, 0))).getTranslation();
        lastSetpointVelocity = new Translation2d(setpoint.velocity, targetDeltaAngle);

        double thetaSetpointVelocity = Math.abs((targetPose.get().getRotation().minus(lastTargetRotation)).getDegrees()) < 10 ? (targetPose.get().getRotation().minus(lastTargetRotation)).getRadians() / (Timer.getTimestamp() - lastTime)
         : thetaController.getSetpoint().velocity;

        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), new State(target.getRotation().getRadians(), thetaSetpointVelocity)) + thetaFFScalar * thetaSetpointVelocity;

        if(thetaErrorAbs < thetaController.getPositionTolerance()){
            thetaVelocity = 0.0;
        }
        lastTargetRotation = targetPose.get().getRotation();
        lastTime = Timer.getTimestamp();

        if(drive != null){
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, targetDeltaAngle));
        }
    }


        @Override
        public void end(boolean interrupted) {
            runningPathing = false;
            drive.stop();
        }
}
