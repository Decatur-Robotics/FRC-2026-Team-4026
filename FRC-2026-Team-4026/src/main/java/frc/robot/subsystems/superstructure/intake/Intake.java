package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake {


    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public Intake(IntakeIO io){
        this.io = io;

        
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Deploy Position", getDeployPosition());
        Logger.recordOutput("Deploy Voltage", getDeployVoltage());
        Logger.recordOutput("Intake Voltage", getIntakeVoltage());

        Logger.recordOutput("Deploy Current", getDeployCurrent());
        Logger.recordOutput("Intake Current", getIntakeCurrent());
    }

    public double getDeployPosition(){
        return inputs.intakeData.deployPosition();

    }
    public double getDeployVoltage(){
        return inputs.intakeData.deployVoltage();
    }
    public double getIntakeVoltage(){

        return inputs.intakeData.intakeVoltage();
    }
    public double getDeployCurrent(){
        return inputs.intakeData.deployCurrent();

    }
    public double getIntakeCurrent(){
        return inputs.intakeData.intakeCurrent();
    }

    public Command deployIntakeCommand(double position){

        return Commands.runOnce(() -> io.setDeployPosition(position));

    }
    public Command retractIntakeCommand(double position){

        return Commands.runOnce(() -> io.setDeployPosition(position));
    }
    public Command runIntakeCommand(double voltage){


        return Commands.startEnd(()-> io.setIntakeVoltage(voltage), () -> io.stopIntake());
        
        
    }
    public Command zeroCommand(double position, double voltage){
        return Commands.runOnce(() -> {io.setIntakeVoltage(0); io.setDeployPosition(position);});

    }



    
    
}

