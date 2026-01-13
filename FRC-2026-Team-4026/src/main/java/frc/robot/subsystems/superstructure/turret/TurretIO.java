package frc.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {
        public TurretIOData turretData = new TurretIOData(
            false,
            0.0,
            0.0,
            0.0
        );
    }

    public record TurretIOData(
        boolean motorConnected,
        double turretPositionDegrees,
        double turretVoltage,
        double turretCurrentAmps
    ) {
    }

    default void updateInputs(TurretIOInputs inputs) {
    }

    default void periodic() {
    }
    
    default void setPosition(double turretPosition) {
    }

    default void stop(){
        
    }

    default void setVoltage(double turretVoltage) {
    }
}
