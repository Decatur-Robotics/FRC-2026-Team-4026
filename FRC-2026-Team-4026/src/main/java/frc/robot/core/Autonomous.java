package frc.robot.core;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;
    public Autonomous(RobotContainer robotContainer, Superstructure superstructure, Drive swerve) {
        this.robotContainer = robotContainer;
        this.superstructure = superstructure;
        this.swerve = swerve;
        registerNamedCommands();
    }


    public void registerNamedCommands() {
        // final Superstructure superstructure = robotContainer.getSuperStructure();
        // NamedCommands.registerCommand("Shoot", superstructure.shootCommand(0, 0));
        // NamedCommands.registerCommand("Intake", superstructure.intakeCommand());

    }   
}
