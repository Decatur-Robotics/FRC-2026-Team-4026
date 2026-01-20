package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.climber.Climber;

public class ClimbCommand extends Command {

    private final Climber subsystem;
    
    public ClimbCommand(Climber climber) {
        this.subsystem = climber;
        this.subsystem.setVoltage(5);
    }

}
