package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
public class Climber extends SubsystemBase {
	
	private final ClimberIO io;
	private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
	private RobotContainer robotContainer = RobotContainer.getInstance();
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


	public Command climberUpCommand(){
		return Commands.runOnce(()->setPositionCommand(ClimberConstants.CLIMBER_UP_POSITION));
	}

	public Command climberDownCommand(){
		return Commands.runOnce(()->setPositionCommand(ClimberConstants.CLIMBER_DOWN_POSITION));
	}


	public Command zeroCommand(){
		return Commands.sequence(
			setPositionCommand(position),
			setVoltageCommand(voltage));
	}

}
