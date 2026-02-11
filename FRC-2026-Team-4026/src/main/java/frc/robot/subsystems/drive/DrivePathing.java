package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public class DrivePathing {
    private final Drive drive;
    private final Supplier<Pose2d> targetPose;

    public DrivePathing(Drive drive, Supplier<Pose2d> targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
    }
}
