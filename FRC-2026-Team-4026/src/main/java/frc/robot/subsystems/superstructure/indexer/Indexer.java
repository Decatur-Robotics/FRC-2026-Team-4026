package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.indexer.IndexerIO.IndexerIOInputs;
import frc.robot.subsystems.superstructure.indexer.IndexerIOInputsAutoLogged;

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
        Logger.recordOutput("Mechanum current and Voltage:", getMechanumCurrent()+", "+getMechanumVoltage());
        Logger.recordOutput("Belt Current and Voltage:", getBeltCurrent()+", "+getBeltVoltage());
        Logger.recordOutput("Kick Current and Voltage:", getKickCurrent()+", "+getKickVoltage());
        if (isEStopped){
            io.stop();
        }
        if(getMechanumCurrent() > IndexerConstants.MAX_CURRENT && getMechanumVelocity() == 0 || getBeltCurrent() > IndexerConstants.MAX_CURRENT && getBeltVelocity() == 0 || getKickCurrent() > IndexerConstants.MAX_CURRENT && getKickVelocity() == 0 ){
            setVoltageCommand(-6);
            Commands.waitSeconds(0.7);
            setVoltageCommand(6);
        }
    }
    
    public Command setVoltageCommand(double voltage){
        
    return this.runOnce(() -> {
            io.setVoltage(voltage);
        });
    }

    public double getMechanumCurrent(){
        return inputs.indexerData.mechanumCurrent();
    }

    public double getBeltCurrent(){
        return inputs.indexerData.beltCurrent();
    }
    public double getKickCurrent(){
        return inputs.indexerData.kickCurrent();
    }

    public double getMechanumVoltage(){
        return inputs.indexerData.mechanumVoltage();
    }

    public double getBeltVoltage(){
        return inputs.indexerData.beltVoltage();
    }

    public double getKickVoltage(){
        return inputs.indexerData.kickVoltage();
    }


    public double getMechanumVelocity(){
        return inputs.indexerData.mechanumVelocity();
}

    public double getBeltVelocity(){
        return inputs.indexerData.beltVelocity();
    }    

    public double getKickVelocity(){
        return inputs.indexerData.kickVelocity();
    }    
}