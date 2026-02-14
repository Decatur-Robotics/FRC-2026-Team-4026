package frc.robot.core;

import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NetworkTables;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.AllianceFlipUtil;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;
    private NetworkTables networkTables;
    public Autonomous(RobotContainer robotContainer, Superstructure superstructure, Drive swerve) {
        this.robotContainer = robotContainer;
        this.superstructure = superstructure;
        this.swerve = swerve;
        networkTables = new NetworkTables();
        registerNamedCommands();
    }


    public void registerNamedCommands() {
         Superstructure superstructure = robotContainer.getSuperstructure();
         NamedCommands.registerCommand("Shoot", superstructure.shootCommand());
         NamedCommands.registerCommand("Intake", superstructure.intakeCommand());

    }   

    public void changePath(){

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(1,1,Rotation2d.fromDegrees(0))
        );
        PathConstraints constraints = new PathConstraints(null, null, null, null);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0, null)
        );

        path.preventFlipping = AllianceFlipUtil.shouldFlip();
    }
    public void changePathOverideFeedback(){
        double robotAngle = swerve.getRotation().getRadians() + (networkTables.getFuelRotation()*Math.PI/180);
        // temporary till find velocity
        double robotVelocity = 2;
        PPHolonomicDriveController.overrideXFeedback(()-> {
            return Math.cos(robotAngle) *robotVelocity;
        });
        PPHolonomicDriveController.clearXFeedbackOverride();

        PPHolonomicDriveController.overrideYFeedback(()-> {
            return Math.sin(robotAngle)*robotVelocity;
        });
        PPHolonomicDriveController.clearYFeedbackOverride();

        PPHolonomicDriveController.overrideRotationFeedback(()->{
            return robotAngle;
        });
        PPHolonomicDriveController.clearRotationFeedbackOverride();


        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    public Command changePathOverideFeedbackCommand(){
        return Commands.runOnce(()->changePathOverideFeedback());
    }
}