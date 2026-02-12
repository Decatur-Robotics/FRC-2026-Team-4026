package frc.robot.subsystems.superstructure.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {

    @AutoLog    
    public class HopperIOInputs {
    public HopperIOData data = new HopperIOData(false, false, 0.0,0.0,0.0,0.0,0.0,0.0);
    public Voltage leftVoltage;
}

public record HopperIOData(
    boolean motorLeftConnected,
    boolean motorRightConnected,
    double leftVoltage,
    double rightVoltage,
    double supplyCurrentLeft,
    double supplyCurrentRight,
    double leftTemp,
    double rightTemp
)
{}
default void setVoltage (double voltage)
{}
default void periodic ()
{}
default void updateInputs (HopperIOInputs inputs)
{}

}
