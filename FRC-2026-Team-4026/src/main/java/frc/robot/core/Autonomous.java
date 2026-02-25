package frc.robot.core;


import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;
    public Autonomous(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // superstructure = robotContainer.getSuperstructure();
        swerve = robotContainer.getDrive();
        eventTriggers();
    }

    public void eventTriggers() {
    //    NamedCommands.registerCommand("Intake", superstructure.intakeCommand().finallyDo(() -> superstructure.storeCommand()));
    //    NamedCommands.registerCommand("Shoot", superstructure.shootCommand().finallyDo(() -> superstructure.storeCommand()));
    //    NamedCommands.registerCommand("Store", superstructure.storeCommand());
    //    NamedCommands.getCommand("Intake");
    }

}
