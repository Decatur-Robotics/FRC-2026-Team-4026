package frc.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs{
         public IndexerIOData indexerData = new IndexerIOData(
            false,
            false,
            0.0,
            0.0,
            0.0,
            0.0);
    }

    public record IndexerIOData(
        boolean rightMotorConnected,
        boolean leftMotorConnected,
        double rightMotorVoltage,
        double leftMotorVoltage,
        double leftCurrent,
        double rightCurrent
    ){}

    default void updateInputs(IndexerIOInputs inputs){}

    default void setVoltage(double voltage){}

    default void periodic(){}

    default void stop(){}
}        
