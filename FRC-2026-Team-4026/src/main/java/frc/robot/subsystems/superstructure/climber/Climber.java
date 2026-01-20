package frc.robot.subsystems.superstructure.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Climber extends SubsystemBase {
	
	private final ClimberIO io;
	private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	private double voltage;
	private double position;

	private final ClimberTalonFX motor;

	public Climber(ClimberIO io) {
		this.io = io;
		this.position = ClimberConstants.CLIMBER_STARTING_POSITION;
		this.motor = new ClimberTalonFX(Ports.CLIMBER_MOTOR_LEFT, Ports.CLIMBER_MOTOR_RIGHT);
		
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
		motor.setVoltage(voltage);
	}

	public void setPosition(double position) {
		this.position = position;
		motor.setPosition(position);
	}

}
