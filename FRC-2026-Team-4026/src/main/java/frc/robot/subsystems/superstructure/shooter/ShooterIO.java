package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
public class ShooterIOInputs { 
    public ShooterIOData data = new ShooterIOData (false,0.0,0.0,0.0,0.0);
    public Voltage leftVoltage;
}

public record ShooterIOData ( 
    
    boolean motorConnected,
    double velocity,
    double voltage,
    double supplyCurrent,
    double temp
    
)
{}

default void setVelocity (double velocity)
{}
default void setVoltage (double voltage)
{}
default void periodic ()
{}
default void updateInputs (ShooterIOInputs inputs)
{}

}