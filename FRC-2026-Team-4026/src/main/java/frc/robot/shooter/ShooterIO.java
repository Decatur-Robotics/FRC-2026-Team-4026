package frc.robot.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
public class ShooterIOInputs { 
    public ShooterIOData data = new ShooterIOData (false,false,0.0,0.0,0.0,0.0);
}

public record ShooterIOData ( 
    
    boolean motorConnectedLeft,
    boolean motorConnectedRight,
    double leftVelocity,
    double rightVelocity,
    double leftVoltage,
    double rightVoltage
    
)
{}
default void setVoltage (double voltage)
{}
default void setVelocity (double velocity)
{}

default void updateInputs (ShooterIOInputs inputs)
{}
}
