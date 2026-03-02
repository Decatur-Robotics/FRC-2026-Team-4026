package frc.robot.core;


import static edu.wpi.first.units.Units.Seconds;

import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.shooter.Shooter;

public class Autonomous {
      private Superstructure superstructure;
    public Autonomous(Superstructure superstructure) {
        this.superstructure = superstructure;
        eventTriggers();
    }

    public void eventTriggers() {
       NamedCommands.registerCommand("Intake", Commands.runOnce(() -> Commands.sequence(superstructure.intakeCommand(), Commands.waitTime(Seconds.of(4))).finallyDo(() -> superstructure.storeCommand()), superstructure));
       NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> superstructure.testShootCommand()).finallyDo(() -> superstructure.noTestShootCommand()));
       NamedCommands.registerCommand("Store", Commands.runOnce(() -> superstructure.storeCommand(), superstructure));
    }

}
