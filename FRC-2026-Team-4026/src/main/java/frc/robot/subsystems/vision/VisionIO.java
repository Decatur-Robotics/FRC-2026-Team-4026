package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
VisionIOData visionData = new VisionIOData(false,
         new TargetObservation(new Rotation2d(), new Rotation2d()), new PoseObservation[0], new int[0]);
    }
    public record VisionIOData(
        boolean cameraConnected,
        TargetObservation targetObservation,
        PoseObservation[] poseObservation,
        int[] aprilTagIDs
    ) {
    }

    public record PoseObservation(
        double timeStamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType poseType
    ) {
    }

    enum PoseObservationType {
                MEGATAG_1,
                MEGATAG_2,
                PHOTONVISION
    }

    public record TargetObservation(
        Rotation2d tx,
        Rotation2d ty
    ) {
    }

    default void updateInputs(VisionIOInputs inputs) {
    }
} 
