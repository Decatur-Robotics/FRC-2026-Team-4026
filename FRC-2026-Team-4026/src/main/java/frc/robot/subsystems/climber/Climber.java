package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;

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


    public Command setPositionCommand(double position){
        return Commands.runOnce(() -> io.setPosition(position));
    }
	public Command setVoltageCommand(double voltage){
		return Commands.runOnce(()->io.setVoltage(voltage));
	}


	public Command climbL1Command(){
		return Commands.sequence(
		setPositionCommand(ClimberConstants.CLIMBER_L1_POSITION),
		setPositionCommand(ClimberConstants.CLIMBER_STARTING_POSITION)
		);	
	}

	public Command climbL2Command(){
		return Commands.sequence(
		climbL1Command(),
		setPositionCommand(ClimberConstants.CLIMBER_L2_POSITION),
		setPositionCommand(ClimberConstants.CLIMBER_STARTING_POSITION)
		);	
	}

	public Command climbL3Command(){
		return Commands.sequence(
		climbL2Command(),
		setPositionCommand(ClimberConstants.CLIMBER_L3_POSITION),
		setPositionCommand(ClimberConstants.CLIMBER_STARTING_POSITION)
		);	
	}


	public Command zeroCommand(){
		return Commands.sequence(
			setPositionCommand(position),
			setVoltageCommand(voltage));
	}

}
