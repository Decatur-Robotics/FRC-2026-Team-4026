package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake {
    private IntakeIOTalonFX deployMotor, intakeMotor;




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

    }

    public double getDeployPosition(){
        return deployMotor.deployMotor.getPosition().getValueAsDouble();

    }
    public double getDeployVoltage(){
        return deployMotor.deployMotor.getPosition().getValueAsDouble();
    }
    public double getIntakeVoltage(){

        return intakeMotor.intakeMotor.getPosition().getValueAsDouble();
    }

    public Command deployIntakeCommand(double position){

        return Commands.runOnce(() -> io.setDeployPosition(position));

    }
    public Command retractIntakeCommand(double position){

        return Commands.runOnce(() -> io.setDeployPosition(position));
    }
    public Command runIntakeCommand(){


        return Commands.startEnd(()-> io.setIntakeVoltage(IntakeConstants.INTAKE_VOLTAGE), () -> io.stopIntake());
        
        
    }



    
    
}

