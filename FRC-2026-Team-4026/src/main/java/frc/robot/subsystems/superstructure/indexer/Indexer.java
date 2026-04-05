package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private boolean isEStopped = false;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double desiredVoltage = 10.0;
    private boolean antiJamRunning = false;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if (isEStopped) {
            io.stop();
        }
        if (getMecanumCurrent() >= IndexerConstants.INDEXER_CURRENT_LIMIT) { // hit our current limit
            // are we stalled?
            if (getMecanumVelocity() == 0 || getBeltVelocity() == 0 || getKickVelocity() == 0){
                if (!antiJamRunning) {
                    antiJamRunning = true;
                    // TODO move this definition outside of periodic
                    Command antiJamSequence = Commands.sequence(
                            setVoltageCommand(-6),
                            Commands.waitSeconds(0.7),
                            setVoltageCommand(6)
                    ).finallyDo(() -> antiJamRunning = false);
                    CommandScheduler.getInstance().schedule(antiJamSequence);
                }
            }
        }
    }

    public Command setVoltageCommand(double voltage){
        return this.runOnce(() -> {
            io.setVoltage(voltage);
        });
    }

    public double getMecanumCurrent() {
        return inputs.indexerData.mecanumCurrent();
    }

    public double getBeltCurrent() {
        return inputs.indexerData.beltCurrent();
    }

    public double getKickCurrent() {
        return inputs.indexerData.kickCurrent();
    }

    public double getMecanumVoltage() {
        return inputs.indexerData.mecanumVoltage();
    }

    public double getBeltVoltage() {
        return inputs.indexerData.beltVoltage();
    }

    public double getKickVoltage() {
        return inputs.indexerData.kickVoltage();
    }

    public double getMecanumVelocity() {
        return inputs.indexerData.mecanumVelocity();
    }

    public double getBeltVelocity() {
        return inputs.indexerData.beltVelocity();
    }

    public double getKickVelocity() {
        return inputs.indexerData.kickVelocity();
    }
}