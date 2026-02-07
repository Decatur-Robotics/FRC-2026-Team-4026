package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Hood extends SubsystemBase {
    private double voltage = 0.0;
    private double position;

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private HoodIO io;
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(1.5),Seconds.of(10), (state) -> signalLogger.writeString("state", state.toString())),
    new SysIdRoutine.Mechanism((volts) -> io.setPosition(volts), inputs.hoodData.position(), this));

    public Hood (HoodIO io){
        this.io = io;
        position = HoodConstants.HOOD_START_POSITION;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        Logger.recordOutput("Hood position", getPosition() );
        Logger.recordOutput("Hood voltage", getVoltage() );
    }
    // commands to run io setters outside of file ykyk
    public Command setPositionCommand(double position){
        return Commands.runOnce(()-> io.setPosition(position));
    }

    public Command setVoltageCommand(double voltage){
        return Commands.runOnce(()-> io.setVoltage(voltage));
    }
    
    public double getPosition(){
       return inputs.hoodData.position();
    }

    public double getVoltage(){
        return inputs.hoodData.voltage();
    }
    // zero voltage and position
    public Command zeroCommand(){
        return Commands.sequence(
            Commands.runOnce(()-> {io.setVoltage(voltage);}),
            Commands.runOnce(()->{io.setPosition(position);})
        );
    }
}
