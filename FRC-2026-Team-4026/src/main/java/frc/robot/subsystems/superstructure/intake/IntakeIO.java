package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public IntakeIOData intakeData = new IntakeIOData(false,false,false,0,0,0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0.0);
    }
    public record IntakeIOData(
        boolean intakeMotorConnected,
        boolean deployMotorConnected,
        boolean deployFollowerMotorConnected,
        double intakeVoltage,
        double deployVoltage,
        double deployFollowVoltage,
        double intakeCurrent,
        double deployCurrent,
        double deployFollowCurrent,
        double deployPosition,
        double deployFollowPosition,
        double intakeTemp,
        double deployTemp,
        double deployFollowTemp,
        double deployVelocity,
        double deployAcceleration,
        double encoderPostion

 

    ){}

    default void updateInputs(IntakeIOInputs inputs){}

    default void setIntakeVoltage(double voltage){}

    default void setDeployPosition(double position){}

    default void stopIntake(){}

    default void periodic(){}

    default void setAltDeployPosition(double position){}

    default void setDeployVoltage(double voltage){}

    default void coast(){}

    default void setSlowPosition(){}
    default void updatePosition(){}
    
}
