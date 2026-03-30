package frc.robot.subsystems.vision.ColorVision.HopperVision;

import org.littletonrobotics.junction.AutoLog;
public interface HopperVisionIO{
    @AutoLog
    public class HopperVisionIOInputs {
        HopperVisionIOData hopperVisionData = new HopperVisionIOData(false,0,0);
    }
    public record HopperVisionIOData(
        boolean cameraConnected,
        double ballsInHopper,
        double percentAreaOfHopperFilled)
    {}

    default void updateInputs(HopperVisionIOInputs inputs){

    }
}
    
