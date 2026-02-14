package frc.robot.core;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NetworkTables;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;
    private NetworkTables networktables =  new NetworkTables();
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

     public void changePathOverideFeedback(){
        double robotAngle = swerve.getRotation().getRadians() + (networktables.getFuelRotation()*Math.PI/180);
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
