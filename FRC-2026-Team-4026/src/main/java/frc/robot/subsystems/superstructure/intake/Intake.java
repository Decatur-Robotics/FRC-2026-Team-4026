package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase{

    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.4).per(Second), Volts.of(1.5),Seconds.of(10), (state) -> SignalLogger.writeString("state", state.toString())),
    new SysIdRoutine.Mechanism((volts) -> io.setDeployVoltage(volts.in(Volts)),null, this));
    private boolean osillatingIntakeGoingUp = true;


    public Intake(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Deploy Position", getDeployPosition());
        Logger.recordOutput("Deploy Voltage", getDeployVoltage());
        Logger.recordOutput("Intake Voltage", getIntakeVoltage());
        Logger.recordOutput("Deploy Current", getDeployCurrent());
        Logger.recordOutput("Intake Current", getIntakeCurrent());
    }

    public void osillatingIntake(){
        if (osillatingIntakeGoingUp){
            io.setDeployPosition(IntakeConstants.HALFWAY_INTAKE_POSITION);
            if(MathUtil.applyDeadband(getDeployPosition(),0.8) == IntakeConstants.HALFWAY_INTAKE_POSITION){
                osillatingIntakeGoingUp = false;
            }
        }
        if (!osillatingIntakeGoingUp){
            io.setDeployPosition(IntakeConstants.DEPLOY_INTAKE_POSITION);
            if(MathUtil.applyDeadband(getDeployPosition(),0.8) == IntakeConstants.DEPLOY_INTAKE_POSITION){
                osillatingIntakeGoingUp = true;
            }
        }


    }


    
    public double getDeployPosition(){

        return inputs.intakeData.deployPosition();
    }

    public double getDeployVoltage(){

        return inputs.intakeData.deployVoltage();
    }
    public double getIntakeVoltage(){
        return inputs.intakeData.intakeVoltage();
    }
    public double getDeployCurrent(){

        return inputs.intakeData.deployCurrent();

    }
    public double getIntakeCurrent(){
        return inputs.intakeData.intakeCurrent();
    }


    public Command osillatingIntakeCommand(){
        return Commands.runOnce(()->osillatingIntake());
    }
    public Command slamIntakeCommand(double position){
        return Commands.runOnce(() -> io.setDeployVoltage(position));
    }

    public Command setDeployVoltageCommand(double voltage){
        return Commands.runOnce(() -> io.setDeployVoltage(voltage));
    }

    public Command coastCommand(){
        return Commands.runOnce(() -> io.coast());
    }

    public Command deployIntakeCommand(double position){
        return Commands.runOnce(() -> io.setDeployPosition(position));
    }
    public Command runIntakeCommand(double voltage){

        return Commands.startEnd(()-> io.setIntakeVoltage(voltage), () -> io.stopIntake());
        
    }
    public Command zeroCommand(double position){

        return Commands.runOnce(() -> {io.setIntakeVoltage(0); io.setDeployPosition(position);});

    }

    public Command sysIdQuasistatic (SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic (SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}

