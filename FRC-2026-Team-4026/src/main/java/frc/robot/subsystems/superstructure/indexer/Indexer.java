package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private IndexerIO io;
    private boolean isEStopped = false;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double desiredVoltage = 10.0;

    public Indexer(IndexerIO io) {
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        if (isEStopped){
            io.stop();
        }
        if(getMecanumCurrent() == IndexerConstants.INDEXER_CURRENT_LIMIT && getMecanumVelocity() == 0 || getBeltCurrent() == IndexerConstants.INDEXER_CURRENT_LIMIT && getBeltVelocity() == 0 || getKickCurrent() > IndexerConstants.INDEXER_CURRENT_LIMIT && getKickVelocity() == 0 ){
            setVoltageCommand(-6);
            Commands.waitSeconds(0.7);
            setVoltageCommand(6);
        }
    }
    // Sets the voltage of the indexer.
    public Command setVoltageCommand(double voltage){
        
    return this.runOnce(() -> {
            io.setVoltage(voltage);
        });
    }

    public double getMecanumCurrent(){
        return inputs.indexerData.mecanumCurrent();
    }

    public double getBeltCurrent(){
        return inputs.indexerData.beltCurrent();
    }
    public double getKickCurrent(){
        return inputs.indexerData.kickCurrent();
    }

    public double getMecanumVoltage(){
        return inputs.indexerData.mecanumVoltage();
    }

    public double getBeltVoltage(){
        return inputs.indexerData.beltVoltage();
    }

    public double getKickVoltage(){
        return inputs.indexerData.kickVoltage();
    }


    public double getMecanumVelocity(){
        return inputs.indexerData.mecanumVelocity();
}

    public double getBeltVelocity(){
        return inputs.indexerData.beltVelocity();
    }    

    public double getKickVelocity(){
        return inputs.indexerData.kickVelocity();
    }    
}