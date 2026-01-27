package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
public class ShooterIOInputs { 
    public ShooterIOData data = new ShooterIOData (false,false,0.0,0.0,0.0,0.0,0.0,0.0);
}

public record ShooterIOData ( 
    
    boolean motorConnectedLeft,
    boolean motorConnectedRight,
    double leftVelocity,
    double rightVelocity,
    double leftVoltage,
    double rightVoltage,
    double supplyCurrentLeft,
    double supplyCurrentRight
    
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
