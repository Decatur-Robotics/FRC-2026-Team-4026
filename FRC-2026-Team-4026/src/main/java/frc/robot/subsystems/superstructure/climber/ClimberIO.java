package frc.robot.subsystems.superstructure.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public class ClimberIOInputs {
		public ClimberIOData climberData = new ClimberIOData(
			false,
			0.0,
			0.0,
			0.0
		);
	}

	public record ClimberIOData(boolean motorConnected, double posDeg, double voltage, double amps) {}

	default void updateInputs(ClimberIOInputs inputs) {}
	default void periodic() {}
	default void setPosition(double position) {}
	default void stop() {}
	default void setVoltage(double voltage) {}
}
