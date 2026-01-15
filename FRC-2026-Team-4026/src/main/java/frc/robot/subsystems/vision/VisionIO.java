package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    public class VisionIOInputs {

    }
    public record visionData(
        boolean cameraConnected,
        TargetObservation targetObservation
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
