package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.junction.AutoLog;
public class Hood extends SubsystemBase {
    private double voltage;
    private double position;

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private HoodIOTalonFX motor;
    private HoodIO io;

    public Hood (HoodIO io){
        this.io = io;
        position = HoodConstants.HOOD_START_POSITION;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

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

    public Command zerCommand(){
        return Commands.sequence(
            Commands.runOnce(()-> {io.setVoltage(voltage);}),
            Commands.runOnce(()->{io.setPosition(position);})
        );

    }
}
