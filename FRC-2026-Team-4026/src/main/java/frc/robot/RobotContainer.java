// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.core.Autonomous;
import frc.robot.core.LogitechControllerButtons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.hood.HoodIOSim;
import frc.robot.subsystems.superstructure.hood.HoodIOTalonFX;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;
import frc.robot.subsystems.superstructure.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.leds.ledsConstants;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;
import frc.robot.subsystems.superstructure.turret.shot.ShotOptimizer;
import frc.robot.subsystems.vision.TestVision;
import frc.robot.subsystems.vision.template.Vision;
import frc.robot.subsystems.vision.template.VisionConstants;
import frc.robot.subsystems.vision.template.VisionIOPhotonVision;
import frc.robot.subsystems.vision.template.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TeamColor;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.hood.HoodIO;
import frc.robot.subsystems.superstructure.hood.HoodIOSim;

import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.template.VisionIOSim;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

//test

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   private Superstructure superstructure;
  private Indexer indexer;
  private Intake intake;
  private Shooter shooter;
   private Hood hood;
  private RobotState robotState;
      private InterpolatingDoubleTreeMap targetVelocities;
  //private final Climber climber;
  private  frc.robot.subsystems.superstructure.leds.leds leds = new frc.robot.subsystems.superstructure.leds.leds();
      private Double robotDistance;
  public Drive drive;
  public static Drive driveInstance;
  private SwerveDriveSimulation driveSimulation;
//  private final TestVision vision;
   private Vision vision;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private static RobotContainer instance;
  private PathPlannerAuto auto;

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
  // private Autonomous autonomous;
  public RobotContainer() {

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

    
      instance = this;
      driveInstance = drive;
    // Configure the trigger bindings

     if(Constants.currentMode != Constants.Mode.SIM) {
      hood = new Hood( new HoodIOTalonFX());
      indexer = new Indexer(new IndexerIOTalonFX());
      intake = new Intake(new IntakeIOTalonFX());
      shooter = new Shooter(new ShooterIOTalonFX());
      // climber = new Climber(new ClimberTalonFX());
      driveSimulation = null;
      this.drive = new Drive(
        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
      robotState = new RobotState(drive);
            // distanceEstimator = new DistanceEstimator();
      //    vision = new TestVision(drive);
              vision = new Vision(drive, new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_LEFT_NAME, VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT), new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_RIGHT_NAME, VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT));
      superstructure = new Superstructure(intake, indexer, shooter, hood, leds, robotState, drive);

      
      // autonomous = new Autonomous(superstructure);
    
    }
 else {

      hood = new Hood(new HoodIOSim());
      indexer = new Indexer(new IndexerIOSim());
      shooter = new Shooter(new ShooterIOSim());
      driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3.6, 6, new Rotation2d()));
      intake = new Intake(new IntakeIOSim(driveSimulation));
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
      //climber = new Climber(new ClimberIOSim());
      drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
      // vision = new TestVision(drive);
      // vision = null;
      // distanceEstimator = new DistanceEstimator();
      new Vision(drive, new VisionIOSim(VisionConstants.CAMERA_FRONT_LEFT_NAME, new Transform3d(), driveSimulation::getSimulatedDriveTrainPose),
                                      new VisionIOSim(VisionConstants.CAMERA_FRONT_RIGHT_NAME, new Transform3d(), driveSimulation::getSimulatedDriveTrainPose));
      vision = null;
                                 robotState = new RobotState(drive);
      superstructure = new Superstructure(intake, indexer, shooter, hood, leds, robotState, drive);
            driveInstance = drive;
            // autonomous = new Autonomous(superstructure);

     }

          //  auto = new PathPlannerAuto("Center Rush Right");
      // auto.activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
      // auto.activePath("Center Rush Right shoot").onFalse(superstructure.shootCommand(() -> 45));

         targetVelocities = new InterpolatingDoubleTreeMap();
             targetVelocities.put(2.7, 38.0);
    targetVelocities.put(3.4, 45.0);

    // targetVelocities.put(3.911, 60.0);
    // targetVelocities.put(5.18, 75.0);
     //NamedCommands.registerCommand("Intake", superstructure.intakeCommand().finallyDo(() -> superstructure.storeCommand()));
     resetSimulationField();
         configurePrimaryBindings();
    configureSecondaryBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configurePrimaryBindings() {
    Joystick joystick = new Joystick(0);

        JoystickButton start = new JoystickButton(joystick, LogitechControllerButtons.start);
        JoystickButton back = new JoystickButton(joystick, LogitechControllerButtons.back);

        POVButton down = new POVButton(joystick, LogitechControllerButtons.down);
        POVButton up = new POVButton(joystick, LogitechControllerButtons.up);
        POVButton left = new POVButton(joystick, LogitechControllerButtons.left);
        POVButton right = new POVButton(joystick, LogitechControllerButtons.right);
        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a); 
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b); 
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick,(LogitechControllerButtons.bumperRight));
        
        
            // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
         drive.setDefaultCommand(
              DriveCommands.joystickDrive(
                drive,
                ()-> joystick.getY(),
                ()-> joystick.getX(),
                ()-> -joystick.getTwist()
            ));

          y.whileTrue(drive.runOnce(() -> drive.setPose(new Pose2d(3.5, 6, new Rotation2d(0, 0)))));
          a.whileTrue(drive.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY(), new Rotation2d(0, 0)))));
          x.whileTrue(drive.setRotationXCommand());
          //b.whileTrue(drive.driveToPoseTeleop(() -> drive.getChassisSpeeds(), () -> new Pose2d(drive.getPose().getX(), drive.getPose().getY(), new Rotation2d(robotState.getTurretRotation()))));
          triggerLeft.whileTrue(DriveCommands.joystickDrive(
                drive,
                ()-> joystick.getY()*0.6,
                ()-> joystick.getX()*0.6,
                ()-> -joystick.getTwist()*0.6));
           b.whileTrue(drive.driveToPoseTeleop(() -> drive.getChassisSpeeds(), () -> new Pose2d( 2.5,  6, new Rotation2d(0,0))));
          // a.whileTrue()
  }

  private void configureSecondaryBindings() {
    Joystick joystick = new Joystick(1);

        JoystickButton start = new JoystickButton(joystick, LogitechControllerButtons.start);
        JoystickButton back = new JoystickButton(joystick, LogitechControllerButtons.back);

        POVButton down = new POVButton(joystick, LogitechControllerButtons.down);
        POVButton up = new POVButton(joystick, LogitechControllerButtons.up);
        POVButton left = new POVButton(joystick, LogitechControllerButtons.left);
        POVButton right = new POVButton(joystick, LogitechControllerButtons.right);
        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a);
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b);
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);

        // triggerLeft.whileTrue(superstructure.shootCommand(() -> targetVelocities.get(distanceEstimator.getDistance()))).onFalse(superstructure.storeCommand());
        triggerRight.whileTrue(superstructure.shootCommand(() -> ShotEstimator.getInstance().getTargetVelocity())).onFalse(superstructure.storeCommand());
        bumperLeft.whileTrue(superstructure.passCommand()).onFalse(superstructure.storeCommand());
        a.whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
        y.whileTrue(intake.deployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
       b.whileTrue(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));
        x.whileTrue(superstructure.dumpCommand()).onFalse(superstructure.storeCommand());
      left.whileTrue(intake.deployIntakeCommand(7));
      up.whileTrue(intake.altDeployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
      down.whileTrue(intake.altDeployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void chooseAuto(){
     if(autoSide.getSelected() == AutoSide.Left){
      if(autoType.getSelected() == AutoType.CenterRush){
            auto = new PathPlannerAuto("Center Rush");
            auto.activePath("Center Rush").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
            auto.activePath("Center rush shoot").onFalse(superstructure.shootCommand(() -> 45));
      } else if(autoType.getSelected() == AutoType.Depot){
          auto = new PathPlannerAuto("Depot Auto");
            auto.activePath("Depot Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
            auto.activePath("Depot to Shoot").onFalse(superstructure.shootCommand(() -> 55));
      } else if(autoType.getSelected() == AutoType.Preload){
            auto = new PathPlannerAuto("Sad Auto");
            auto.activePath("Sad Path").onFalse(superstructure.shootCommand(() -> 55));
      }
     } else if(autoSide.getSelected() == AutoSide.Center){
                  auto = new PathPlannerAuto("Center Depot Auto");
            auto.activePath("Center Depot Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
            auto.activePath("Center Depot Shoot").onFalse(superstructure.shootCommand(() -> 50));
     } else if(autoSide.getSelected() == AutoSide.Right){
      if(autoType.getSelected() == AutoType.CenterRush){
        auto = new PathPlannerAuto("Center Rush Right");
      auto.activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
      auto.activePath("Center Rush Right shoot").onFalse(superstructure.shootCommand(() -> 45));
      } else if(autoType.getSelected() == AutoType.Depot){
         auto = new PathPlannerAuto("HP Auto Right");
            auto.activePath("Start right to HP").onTrue(superstructure.storeCommand());
            auto.activePath("HP Shoot").onFalse(superstructure.shootCommand(() -> 55));
      }
     }
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return superstructure.shootCommand(() -> 42);
      // return Commands.sequence(Commands.run(() -> drive.setPose(new Pose2d(3.56, 6, new Rotation2d()))),
      //  Commands.parallel(drive.driveToPoseAuto(() -> new Pose2d(0.6,6,new Rotation2d()))),
      //  superstructure.intakeCommand().until(() -> drive.isAligned()).andThen(superstructure.storeCommand()),
      //   drive.driveToPoseAuto(() ->  new Pose2d(0.8, 6, new Rotation2d(Units.degreesToRadians(-27)))), superstructure.shootCommand(() -> 55));
      // return new PathPlannerAuto("New Auto");
    //  return superstructure.testShootCommand().finallyDo(() -> superstructure.noTestShootCommand());
  }

  public Command pathfinderToPose(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(4.69, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    Command pathfinderCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfinderCommand;
  }


  public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3.55, 6, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }


  public void updateSimulation(){
    if(Constants.currentMode != Constants.Mode.SIM) return;
      
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

    public void distanceUpdate(){
        configureSecondaryBindings();
  }

   public double getTargetVelocity(){
    return targetVelocities.get(robotDistance);
 }
 
  public static RobotContainer getInstance(){
    return instance;
  }

  public static Drive getDrive(){
    return driveInstance;
  }

  public Pose2d getDrivePose(){
    return drive.getPose();
  }
}
