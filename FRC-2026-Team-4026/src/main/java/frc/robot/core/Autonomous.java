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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;

public class Autonomous {
      private enum AutoType{
  CenterRush("Center Rush"), Depot("Depot"), Preload("Preload");

  private String autoName;
  private AutoType(String autoName){
    this.autoName = autoName;
  }
}

private enum AutoSide{
  Left("Left"), Center("Center"), Right("Right");

  private String autoName;
  private AutoSide(String autoName){
    this.autoName = autoName;
  }
}

    private SendableChooser<AutoSide> autoSide;
  private SendableChooser<AutoType> autoType;
        private Superstructure superstructure;
      private PathPlannerAuto auto;

    public Autonomous(Superstructure superstructure) {
            this.auto = new PathPlannerAuto("Center Rush");
        this.superstructure = superstructure;
            autoSide = new SendableChooser<>();
    autoSide.setDefaultOption(AutoSide.Left.autoName, AutoSide.Left);
    autoSide.addOption(AutoSide.Center.autoName, AutoSide.Center);
    autoSide.addOption(AutoSide.Right.autoName, AutoSide.Right);

    autoType = new SendableChooser<>();
    autoType.setDefaultOption(AutoType.CenterRush.autoName, AutoType.CenterRush);
    autoType.addOption(AutoType.Depot.autoName, AutoType.Depot);
    autoType.addOption(AutoType.Preload.autoName, AutoType.Preload);
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add("Side", autoSide);
    autoTab.add("Type", autoType);
    }



      public void chooseAuto(){
     if(autoSide.getSelected() == AutoSide.Left){
        if(autoType.getSelected() == AutoType.CenterRush){
                auto = new PathPlannerAuto("Center Rush");
                auto.activePath("Center Rush").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
                auto.activePath("Center rush shoot").onFalse(superstructure.shootCommand());
        } else if(autoType.getSelected() == AutoType.Depot){
            auto = new PathPlannerAuto("Depot Auto");
                auto.activePath("Depot Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
                auto.activePath("Depot to Shoot").onFalse(superstructure.shootCommand());
        } else if(autoType.getSelected() == AutoType.Preload){
                superstructure.shootCommand();
        }
     } else if(autoSide.getSelected() == AutoSide.Center){
                  auto = new PathPlannerAuto("Center Depot Auto");
            auto.activePath("Center Depot Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
            auto.activePath("Center Depot Shoot").onFalse(superstructure.shootCommand());
     } else if(autoSide.getSelected() == AutoSide.Right){
        if(autoType.getSelected() == AutoType.CenterRush){
            auto = new PathPlannerAuto("Center Rush Right");
        auto.activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
        auto.activePath("Center Rush Right shoot").onFalse(superstructure.shootCommand());
        } else if(autoType.getSelected() == AutoType.Depot){
            auto = new PathPlannerAuto("HP Auto Right");
                auto.activePath("Start right to HP").onTrue(superstructure.storeCommand());
                auto.activePath("HP Shoot").onFalse(superstructure.shootCommand());
        } else if(autoType.getSelected() == AutoType.Preload){
            superstructure.shootCommand();
        }
     }
  }

  public PathPlannerAuto getAuto(){
    return auto;
  }
}
