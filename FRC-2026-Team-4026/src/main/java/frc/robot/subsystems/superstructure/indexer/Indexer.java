package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private IndexerIO io;
    private boolean isEStopped = false;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Left and Right Indexer Current:", getLeftCurrent()+", "+getRightCurrent());
        Logger.recordOutput("Left and Right Indexer Voltage:", getLeftMotorVoltage()+", "+getRightMotorVoltage());
        if (isEStopped){
            io.stop();
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
}
