package frc.robot.core;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;
    private Pose2d m;
    public Autonomous(Drive swerve) {
        this.robotContainer = RobotContainer.getInstance();
        this.superstructure = robotContainer.getSuperstructure();
        this.swerve = swerve;
        registerNamedCommands();
    }


    public void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", superstructure.shootCommand());
        NamedCommands.registerCommand("Intake", superstructure.intakeCommand());
        NamedCommands.registerCommand("Store", superstructure.storeCommand());
    }   
}
