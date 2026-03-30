package frc.robot.core;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;

public class Autonomous {
  private Drive drive;
  private Superstructure superstructure;
  private Command auto;
      private Boolean hopperFull = false;
    private Boolean hopperEmpty = true;
  private enum AutoType{
    DoubleCenter("Double Center"),
    DoubleDepot("Double Center + Depot"),
    Orbit("Orbit"),
    OrbitPass("Orbit Pass");
    private String autoType;
    private AutoType(String autoType){
      this.autoType = autoType;
    }
  }
  private enum Side{
    Left("Left"), Right("Right");
    private String side;
    private Side(String side){
      this.side = side;
    }
  }
  private enum Smart{
    Yes("Yes"), No("No");
    private String smart;
    private Smart(String smart){
      this.smart = smart;
    }
  }


  public Autonomous(Superstructure superstructure, Drive drive){
    this.drive = drive;
    this.superstructure = superstructure;
  }

  public void setAuto(){

  }

  public Command DoubleCenterSwipe(){
    PathPlannerAuto centerSwipe1 = new PathPlannerAuto("Center Rush 1");
    centerSwipe1.activePath("Center Rush Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    centerSwipe1.activePath("Center Rush Shoot").onFalse(autoShootCommand());
    PathPlannerAuto centerSwipe2 = new PathPlannerAuto("Center Test 2");
    centerSwipe2.activePath("Center Rush Intake 2").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    centerSwipe2.activePath("Center Rush 2 Shoot").onFalse(autoShootCommand());
    return Commands.sequence(centerSwipe1, centerSwipe2);
  }

  public Command DoubleCenterDepot(){
        PathPlannerAuto centerSwipe1 = new PathPlannerAuto("Center Rush 1");
    centerSwipe1.activePath("Center Rush Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    centerSwipe1.activePath("Center Rush Shoot").onFalse(autoShootCommand());
    PathPlannerAuto centerSwipe2 = new PathPlannerAuto("Center Test 2");
    centerSwipe2.activePath("Center Rush Intake 2").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    centerSwipe2.activePath("Center Rush 2 Shoot").onFalse(autoShootCommand());
    PathPlannerAuto depot = new PathPlannerAuto("Depot ");
    depot.activePath("Center Depot Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    return Commands.parallel(centerSwipe1, centerSwipe2, Commands.parallel(drive.driveShootCommand(() -> depot.getStartingPose()), superstructure.shootOnMoveCommand()), depot);
  }

  public Command DoubleCenterSmart(){
            PathPlannerAuto centerShoot = new PathPlannerAuto("Center Smart Shoot");
            centerShoot.activePath("Center rush shoot").onFalse(autoShootCommand());
        PathPlannerAuto centerSwipe1 = new PathPlannerAuto("Center Rush 1");
    centerSwipe1.activePath("Center Rush Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
    centerSwipe1.activePath("Center Rush Shoot").onFalse(autoShootCommand());
    PathPlannerAuto centerSwipe2 = new PathPlannerAuto("Center 2 Smart");
    centerSwipe2.activePath("Center 2 Smart").onFalse(Commands.sequence(Commands.parallel(drive.driveToFuel(), superstructure.intakeCommand()).until(()->false), pathfinderToPose(centerShoot.getStartingPose())));
    return Commands.sequence(centerSwipe1, centerSwipe2);
  }

  public Command shootOnMoveTestCommand(){
    return drive.driveShootCommand(()->new Pose2d(3,3,new Rotation2d()));
  }



  public Command pathfinderToPose(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(4.69, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    Command pathfinderCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfinderCommand;
  }

    public Command pathfinderToPoseIntake(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(1.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    Command pathfinderCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfinderCommand;
  }

      public Command autoShootCommand(){
        return superstructure.oscillateShootCommand().until(() -> !hopperEmpty);
    }
}
