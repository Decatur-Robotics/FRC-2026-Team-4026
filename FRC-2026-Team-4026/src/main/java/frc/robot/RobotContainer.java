// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.core.Autonomous;
import frc.robot.core.LogitechControllerButtons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
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
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.TeamColor;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;


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
  //  public ShotEstimator shotEstimator;
  private RobotState robotState;
      private InterpolatingDoubleTreeMap targetVelocities;
  //private final Climber climber;
  private static frc.robot.subsystems.superstructure.leds.leds leds = new frc.robot.subsystems.superstructure.leds.leds();
      private Double robotDistance;
  public Drive drive;
  public static Drive driveInstance;
  private SwerveDriveSimulation driveSimulation;
   private Vision vision;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private static RobotContainer instance;
  private PathPlannerAuto auto;


   private Autonomous autonomous;
  public RobotContainer() {

      instance = this;


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
              vision = new Vision(drive, new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_LEFT_NAME, VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT), new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_RIGHT_NAME, VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT));
      superstructure = new Superstructure(intake, indexer, shooter, hood, leds, robotState, drive);

            driveInstance = drive;
       autonomous = new Autonomous(superstructure);
    
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
      new Vision(drive, new VisionIOPhotonVisionSim(VisionConstants.CAMERA_FRONT_LEFT_NAME, new Transform3d(), driveSimulation::getSimulatedDriveTrainPose),
                                      new VisionIOPhotonVisionSim(VisionConstants.CAMERA_FRONT_RIGHT_NAME, new Transform3d(), driveSimulation::getSimulatedDriveTrainPose));
                                 robotState = new RobotState(drive);
      superstructure = new Superstructure(intake, indexer, shooter, hood, leds, robotState, drive);
            driveInstance = drive;
            autonomous = new Autonomous(superstructure);
     }

  

         targetVelocities = new InterpolatingDoubleTreeMap();
             targetVelocities.put(2.7, 38.0);
    targetVelocities.put(3.4, 45.0);

    // targetVelocities.put(3.911, 60.0);
    // targetVelocities.put(5.18, 75.0);
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
                ()-> -joystick.getY(),
                ()-> -joystick.getX(),
                ()-> -joystick.getTwist()
            ));


          // y.whileTrue(drive.runOnce(() -> drive.setPose(new Pose2d(3.5, 6, new Rotation2d(0, 0)))));
          // a.whileTrue(drive.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY(), new Rotation2d(0, 0)))));
          // x.whileTrue(drive.setRotationXCommand());
          bumperRight.whileTrue(superstructure.alignCommand()).onFalse(leds.setAllLedsCommand(ledsConstants.BLUE));
          bumperLeft.whileTrue(drive.alignHubPathpl());
          triggerLeft.whileTrue(DriveCommands.joystickDrive(
                drive,
                ()-> -joystick.getY()*0.6,
                ()-> -joystick.getX()*0.6,
                ()-> -joystick.getTwist()*0.6));
          //  b.whileTrue(drive.driveToPoseTeleop(() -> drive.getChassisSpeeds(), () -> new Pose2d( 2.5,  6, new Rotation2d(0,0))));
  }

  public void configureSecondaryBindings() {
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

      //   triggerLeft.whileTrue(superstructure.shootCommand(() -> 42.0)).onFalse(superstructure.storeCommand());
      //   triggerRight.whileTrue(superstructure.shootCommand()).onFalse(superstructure.storeCommand());
      //   // bumperLeft.whileTrue(superstructure.passCommand()).onFalse(superstructure.storeCommand());
      //   a.whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
      //   y.whileTrue(intake.deployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
      //  b.whileTrue(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));
      //   x.whileTrue(superstructure.dumpCommand()).onFalse(superstructure.storeCommand());
      // left.whileTrue(intake.deployIntakeCommand(7));
      //  right.whileTrue(intake.oscillateIntakeCommand());

    a.whileTrue(leds.setAllLedsCommand(ledsConstants.BLUE));
    b.whileTrue(leds.flashAllLedsCommand(ledsConstants.RED, 5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
      // return autonomous.getAuto();
      PathPlannerAuto auto = new PathPlannerAuto("Center Rush Right");
      auto.activePath("Center Rush Right Intake").whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
      auto.activePath("Center Rush Right shoot").onFalse(Commands.parallel(superstructure.shootCommand(), intake.oscillateIntakeCommand()));
      return leds.setAllLedsCommand(ledsConstants.BLUE);

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
