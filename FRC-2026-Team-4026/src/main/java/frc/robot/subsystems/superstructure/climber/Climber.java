package frc.robot.subsystems.superstructure.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	
	private final ClimberIO io;
	private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	private double voltage = 0.0;
	private double position = 0.0;

	public Climber(ClimberIO io) {
		this.io = io;
		this.position = ClimberConstants.CLIMBER_STARTING_POSITION;
	}

	@Override
	public void periodic() {

		io.updateInputs(inputs);
		Logger.processInputs("Climber", inputs);
		Logger.recordOutput("Climber Position", position);
		Logger.recordOutput("Climber Voltage", voltage);

	}

	public void setVoltage(double voltage) {
		this.voltage = voltage;
		io.setVoltage(voltage);
	}

	public void setPosition(double position) {
		this.position = position;
		io.setPosition(position);
	}

}
