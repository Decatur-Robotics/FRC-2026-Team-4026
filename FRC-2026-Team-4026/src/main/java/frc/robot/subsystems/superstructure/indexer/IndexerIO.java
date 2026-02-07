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
        Boolean mechanumMotorConnected,
        Boolean beltMotorConnected,
        Boolean kickMotorConnected,
        double mechanumVoltage,
        double beltVoltage,
        double kickVoltage,
        double mechanumCurrent,
        double beltCurrent,
        double kickCurrent,
        double mechanumVelocity,
        double beltVelocity,
        double kickVelocity,
        double mechanumTemp,
        double beltTemp,
        double kickTemp
    ){}

    default void updateInputs(IndexerIOInputs inputs){}

    default void setVoltage(double voltage){}

    default void periodic(){}

    default void stop(){}
    
}        
