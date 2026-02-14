package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import lombok.extern.java.Log;


public class DrivePathing {
    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    private static final LoggedTunableNumber DrivekP = new LoggedTunableNumber("DrivePathing/kP", 1.0);

    private static final LoggedTunableNumber DrivekD = new LoggedTunableNumber("DrivePathing/kD", 0.0);
    private static final LoggedTunableNumber ThetakP = new LoggedTunableNumber("DrivePathing/Theta/kP", 1.0);
    private static final LoggedTunableNumber ThetakD = new LoggedTunableNumber("DrivePathing/Theta/kD", 0.0);

    private static final LoggedTunableNumber driveMaxSpeed = new LoggedTunableNumber("DrivePathing/maxSpeed", 1.0);
    private static final LoggedTunableNumber driveMaxAngularSpeed = new LoggedTunableNumber("DrivePathing/maxAngularSpeed", 1.0);
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber("DrivePathing/maxAcceleration", 1.0);
    private static final LoggedTunableNumber driveMaxAngularAcceleration = new LoggedTunableNumber("DrivePathing/maxAngularAcceleration", 1.0);

    private TrapezoidProfile driveProfile;
    private final PIDController driveController = new PIDController(0,0,0,0.02);
    private final PIDController thetaController = new PIDController(0,0,0, 0.02);
    public DrivePathing(Drive drive, Supplier<Pose2d> targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

    }

    public void initialize() {
        Pose2d currentPose = drive.getPose();
        Pose2d target = targetPose.get();
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        Translation2d linearFieldSpeeds = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(driveMaxSpeed.get(), driveMaxAcceleration.get()));
        
        driveController.reset();
       // thetaController.reset(currentPose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);

        
    }

    public void execute(){

    }
}
