package frc.robot.core;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;


public class Autonomous {
  private Drive drive;
  private Superstructure superstructure;
  private enum AutoType{
    CenterRush("Center Rush"),
    DoubleCenter("Double Center"),
    DoubleDepot("Double Center + Depot"),
    Passing("Passing"),
    Disrupt("Disrupt");
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



  public Autonomous(Superstructure superstructure, Drive drive){
    this.drive = drive;
    this.superstructure = superstructure;
  }

  public void setAuto(){

  }

  public Command DoubleCenterLoop(){
        PathPlannerAuto auto1 = new PathPlannerAuto("Center Rush Right");
    auto1
        .activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand())
        .onFalse(superstructure.storeCommand());

    PathPlannerAuto auto2 = new PathPlannerAuto("Center Right 2 Test");
    auto2
        .activePath("Center Rush 2 1002").whileTrue(superstructure.intakeCommand())
        .onFalse(superstructure.storeCommand());
    return Commands.sequence(auto1,
        superstructure.oscillateShootCommand().withTimeout(Seconds.of(3)), superstructure.storeCommand().withTimeout(0.1), auto2, superstructure.oscillateShootCommand());
  }

  public Command DoubleCenterLoopLeft(){
    PathPlannerAuto auto1 = new PathPlannerAuto("Center Rush Right", true);
    auto1
        .activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand())
        .onFalse(superstructure.storeCommand());

    PathPlannerAuto auto2 = new PathPlannerAuto("Center Right 2 Test", true);
    auto2
        .activePath("Center Rush 2 1002").whileTrue(superstructure.intakeCommand())
        .onFalse(superstructure.storeCommand());
    return Commands.sequence(auto1,
        superstructure.oscillateShootCommand().withTimeout(Seconds.of(3)), superstructure.storeCommand().withTimeout(0.1), auto2, superstructure.oscillateShootCommand());
  }

  public Command passingAuto(){
    PathPlannerAuto auto = new PathPlannerAuto("Passing");
    auto.activePath("Passing Intake 1").onTrue(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 1").whileTrue(superstructure.passIntakeCommand())
    .onFalse(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 2").whileTrue(superstructure.passIntakeCommand())
    .onFalse(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 3").whileTrue(superstructure.passIntakeCommand());
    return auto;
  }

    public Command passingAutoLeft(){
    PathPlannerAuto auto = new PathPlannerAuto("Passing", true);
    auto.activePath("Passing Intake 1").onTrue(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 1").whileTrue(superstructure.passIntakeCommand())
    .onFalse(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 2").whileTrue(superstructure.passIntakeCommand())
    .onFalse(superstructure.intakeCommand());
    auto.activePath("Passing Shoot 3").whileTrue(superstructure.passIntakeCommand());
    return auto;
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
    depot.activePath("Center Depot Shoot").onFalse(superstructure.oscillateShootCommand());
    return Commands.parallel(centerSwipe1, centerSwipe2, Commands.parallel(pathfinderToPose(depot.getStartingPose()), autoShootCommand()), depot);
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
        return Commands.sequence(superstructure.oscillateShootCommand(), Commands.waitTime(Seconds.of(3)));
    }


}
