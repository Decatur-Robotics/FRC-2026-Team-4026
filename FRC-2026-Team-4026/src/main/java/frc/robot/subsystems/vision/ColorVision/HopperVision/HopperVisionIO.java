package frc.robot.subsystems.vision.ColorVision.HopperVision;

import org.littletonrobotics.junction.AutoLog;
public interface HopperVisionIO{
    @AutoLog
    public class HopperVisionIOInputs {
        HopperVisionIOData hopperVisionData = new HopperVisionIOData(true,0);
    }
    public record HopperVisionIOData(
        boolean cameraConnected,
        double ballsInHopper)
    {}

    default void updateInputs(HopperVisionIOInputs inputs){

    }
}
    
