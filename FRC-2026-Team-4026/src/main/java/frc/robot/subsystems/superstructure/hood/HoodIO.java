package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs{
        public HoodIOData hoodData = new HoodIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    public record HoodIOData(
        boolean motorConnected,
        double voltage,
        double position,
        double supplyAmps,
        double torqueCurrent

    ){}
    default void periodic(){}

    default void updateInputs(HoodIOInputs inputs){}

    default void setPosition(double position){}

    default void setVoltage(double voltage){}

    default void stop(){}

    default void setPID(HoodConstants constants){}
}