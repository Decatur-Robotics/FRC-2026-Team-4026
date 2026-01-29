package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public IntakeIOData intakeData = new IntakeIOData(false,false,0,0,0,0,0);
    
    }
    record IntakeIOData(
        boolean intakeMotorConnected,
        boolean deployMotorConnected,
        double intakeVoltage,
        double deployVoltage,
        double intakeCurrent,
        double deployCurrent,
        double deployPosition

 

    ){}

    default void updateInputs(IntakeIOInputs inputs){


    }
    default void setIntakeVoltage(double voltage){

    }


    default void setDeployPosition(double position){
        
    }
    default void stopIntake(){

    }
    default void periodic(){

        
    }
}
