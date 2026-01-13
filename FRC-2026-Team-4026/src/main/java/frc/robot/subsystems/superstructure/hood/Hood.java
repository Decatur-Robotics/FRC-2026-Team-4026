package frc.robot.subsystems.superstructure.hood;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.junction.AutoLog;
public class Hood extends SubsystemBase {
    private double voltage;
    private double position;

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private boolean isEStopped = false;
    private HoodIOTalonFX motor;
    private HoodIO io;

    public Hood (HoodIO io){
        this.io = io;
        
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Hood Inputs", inputs);
        if(isEStopped){
            io.stop();
        }
    }

    public Command setPositionCommand(double position){
        return Commands.runOnce(()-> io.setPosition(position));
    }

    public double getPosition(){
       return HoodIOInputs.hoodData.position();
    }

    public double getVoltage(){
        return voltage;
    }
}
