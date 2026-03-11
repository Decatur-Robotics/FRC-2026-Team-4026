package frc.robot.core;


import static edu.wpi.first.units.Units.Seconds;

import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;

public class Autonomous {
      private Superstructure superstructure;
      private PathPlannerAuto auto;
    public Autonomous(Superstructure superstructure) {
        this.superstructure = superstructure;
        auto = new PathPlannerAuto("Center Rush");
        eventTriggersCenterRush();
    }

    public void eventTriggersCenterRush() {
       NamedCommands.registerCommand("Intake", Commands.runOnce(() -> Commands.sequence(superstructure.intakeCommand(), Commands.waitSeconds(4)).finallyDo(() -> superstructure.storeCommand())));
       NamedCommands.registerCommand("Shoot", superstructure.shootCommand(ShotEstimator.getInstance().getTargetVelocity()).until(() -> !RobotState.isAutonomous()));

       auto.activePath("Center Rush").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    }

}
