package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.AutoLog;

public interface ColorVisionIO{
    @AutoLog
    public class ColorVisionIOInputs {
        ColorVisionIOData colorVisionData = new ColorVisionIOData(true,0);
    }
    public record ColorVisionIOData(
        boolean cameraConnected,
        float averageYaw)
    {}

    default void updateInputs(ColorVisionIOInputs inputs){

    }
}
