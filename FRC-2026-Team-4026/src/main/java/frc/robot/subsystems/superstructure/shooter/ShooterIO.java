package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
@AutoLog
public class ShooterIOInputs { 
    public ShooterIOData data = new ShooterIOData (false,false,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    public Voltage leftVoltage;
}

public record ShooterIOData ( 
    
    boolean motorConnected,
    boolean followerMotorConnected,
    double velocity,
    double followerVelocity,
    double voltage,
    double followerVoltage,
    double supplyCurrent,
    double followerSupplyCurrent,
    double temp,
    double followerTemp
    
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