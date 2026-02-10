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
        final Superstructure superstructure = robotContainer.getSuperstructure();
        NamedCommands.registerCommand("Shoot", superstructure.shootCommand());
        NamedCommands.registerCommand("Intake", superstructure.intakeCommand());
        NamedCommands.registerCommand("Store", superstructure.storeCommand());

    }   
}
