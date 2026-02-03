package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class Indexer extends SubsystemBase{
    private IndexerIO io;
    private boolean isEStopped = false;
    private final String inputsName;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.inputsName = this.getClass().getSimpleName() + "Inputs";
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);
        Logger.recordOutput("Left and Right Current:", getLeftCurrent()+", "+getRightCurrent());
        Logger.recordOutput("Left and Right Voltage:", getLeftMotorVoltage()+", "+getRightMotorVoltage());
        if (isEStopped){
            io.stop();
        }
        if(getLeftCurrent() > IndexerConstants.maxCurrent && getLeftVelocity() == 0 || getRightCurrent() > IndexerConstants.maxCurrent && getRightVelocity() == 0){
            inputs.indexerData.leftVelocity *= -1;
            inputs.indexerData.rightVelocity *= -1;
            Commands.waitSeconds(0.2);
            inputs.indexerData.leftVelocity = *= -1;
            inputs.indexerData.rightVelocity *= -1;
            
        }
    }
    
    public Command setVoltageCommand(double voltage){
    return this.runOnce(() -> {
            io.setVoltage(voltage);
        });
    }

    public double getLeftCurrent(){
        return inputs.indexerData.leftCurrent();
    }

    public double getRightCurrent(){
        return inputs.indexerData.rightCurrent();
    }
    
    public double getLeftMotorVoltage(){
        return inputs.indexerData.leftMotorVoltage();
    }

    public double getRightMotorVoltage(){
        return inputs.indexerData.rightMotorVoltage();
    }

    public double getLeftVelocity(){
        return inputs.indexerData.leftVelocity();
}

    public double getRightVelocity(){
        return inputs.indexerData.rightVelocity();
}