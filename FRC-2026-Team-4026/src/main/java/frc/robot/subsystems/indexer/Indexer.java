package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private double voltage;
    private IndexerIO io;
    private boolean isEStopped = false;
    private final String inputsName;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.inputsName = this.getClass().getSimpleName() + "Inputs";
        this.io = io;
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);

        if (isEStopped){
            io.stop();
        }
    }
    
    public Command setVoltageCommand(double voltage){
    return this.runOnce(() -> {
            this.voltage = voltage;
            io.setVoltage(voltage);
        });
    }

    public double getMotorVoltage(){
        return voltage;
    }
}
