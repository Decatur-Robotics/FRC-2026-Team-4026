package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

	@AutoLog
	public class ClimberIOInputs {
		public ClimberIOData climberData = new ClimberIOData(
			false,
			false,
			0.0,
			0.0,
			0.0,
			0.0
		);
	}

	public record ClimberIOData(boolean motorConnected,boolean followerMotorConnected, double posDeg, double voltage, double amps,double temp) {}

	default void updateInputs(ClimberIOInputs inputs) {}
	default void periodic() {}
	default void setPosition(double position) {}
	default void stop() {}
	default void setVoltage(double voltage) {}
}
