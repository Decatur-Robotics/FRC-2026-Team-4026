package frc.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private double turretPosition;
    private double turretVoltage;
    private double turretCurrent;

    private TurretIO io;
    
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    public Turret(TurretIO io) {
        this.io = io;
        turretPosition = TurretConstants.TURRET_STARTING_POSITION; 
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret Position", getPosition());
        Logger.recordOutput("Turret Voltage", getVoltage());
        
    }

    public Command setPositionCommand(double position){
        return runOnce(() -> io.setPosition(position));
    }

    public Command setVoltageCommand(double voltage){
        return runOnce(() -> io.setVoltage(voltage));
    }

    public double getVoltage(){
        return inputs.turretData.turretVoltage();
    }

    public double getPosition(){
        return inputs.turretData.turretPositionDegrees();
    }

    public Command zeroCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> {io.setVoltage(turretVoltage);}),
            Commands.runOnce(() -> {io.setPosition(0.0);})
        );
    }
    
}
