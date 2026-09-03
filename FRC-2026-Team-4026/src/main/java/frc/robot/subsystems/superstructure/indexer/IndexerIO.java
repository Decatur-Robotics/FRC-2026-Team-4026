package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs{
         public IndexerIOData indexerData = new IndexerIOData(
            false,
            false,
            false,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
            );

    }

    public record IndexerIOData(
        boolean mechanumMotorConnected,
        boolean beltMotorConnected,
        boolean kickMotorConnected,
        double mecanumVoltage,
        double beltVoltage,
        double kickVoltage,
        double mecanumCurrent,
        double beltCurrent,
        double kickCurrent,
        double mecanumVelocity,
        double beltVelocity,
        double kickVelocity,
        double mecanumTemp,
        double beltTemp,
        double kickTemp
    ){}

    default void updateInputs(IndexerIOInputs inputs){}

    default void setVoltage(double voltage){}

    default void periodic(){}

    default void stop(){}

}        
