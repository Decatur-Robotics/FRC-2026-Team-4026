// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.subsystems.superstructure.leds.Leds;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;
import frc.robot.subsystems.superstructure.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.leds.Leds;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.ColorVision.ColorVision;
import frc.robot.subsystems.vision.ColorVision.ColorVisionConstants;
import frc.robot.subsystems.vision.ColorVision.ColorVisionIOPhotonVision;
import frc.robot.subsystems.vision.ColorVision.HopperVision.HopperVision;
import frc.robot.subsystems.vision.ColorVision.HopperVision.HopperVisionConstants;
import frc.robot.subsystems.vision.ColorVision.HopperVision.HopperVisionIOPhotonVision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import frc.robot.util.LoggedTunableNumber;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// test

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private HopperVision hopperVision;
        private Superstructure superstructure;
        private Indexer indexer;
        private Intake intake;
        private Shooter shooter;

        // enum to make sure we get our autos right
        private enum AutoType {
                CenterRush("Center Rush"), Depot("Depot"), Preload("Preload"), DoubleCenter("Double Center"),
                RushDepot("Center Rush + Depot");

                private String autoName;

                private AutoType(String autoName) {
                        this.autoName = autoName;
                }
        }

        private enum AutoSide {
                Left("Left"), Center("Center"), Right("Right");

                private String autoName;
         

                private AutoSide(String autoName) {
                        this.autoName = autoName;
                }
        }

        private Command autoCommand;
        private SendableChooser<AutoSide> autoSide;
        private SendableChooser<AutoType> autoType;

        private InterpolatingDoubleTreeMap targetVelocities;

        private Leds leds;
        private Double robotDistance;
        public Drive drive;
        public static Drive driveInstance;
        private SwerveDriveSimulation driveSimulation;
        private Vision vision;
        private ColorVision colorVision;
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        private static RobotContainer instance;
        private boolean shootingOnMove;

        private Autonomous autonomous;

        public RobotContainer() {

                instance = this;
                shootingOnMove = false;
                leds = new Leds();
                hopperVision = new HopperVision(
                                new HopperVisionIOPhotonVision(HopperVisionConstants.HOPPER_CAMERA_NAME));
                ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
                // creates our subsystems if we're on the robot
                if (Constants.currentMode != Constants.Mode.SIM) {
                        indexer = new Indexer(new IndexerIOTalonFX());
                        intake = new Intake(new IntakeIOTalonFX());
                        shooter = new Shooter(new ShooterIOTalonFX());

                        driveSimulation = null;
                        tab.add("Indexer Voltage", indexer.setVoltageCommand(0))
                                        .withWidget(BuiltInWidgets.kNumberSlider);
                        this.drive = new Drive(
                                        new GyroIOPigeon2(),
                                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                                        (pose) -> {
                                        });
                        vision = new Vision(drive,
                                        new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_LEFT_NAME,
                                                        VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT),
                                        new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_RIGHT_NAME,
                                                        VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT));
                        colorVision = new ColorVision(drive,
                                        new ColorVisionIOPhotonVision(ColorVisionConstants.FIELD_CAMERA_NAME,
                                                        ColorVisionConstants.FIELD_CAMERA_TO_ROBOT));
                        superstructure = new Superstructure(intake, indexer, shooter, leds);

                        driveInstance = drive;
                        autonomous = new Autonomous(superstructure, drive);

                } else {
                        // this is if we're testing in simulation

                        indexer = new Indexer(new IndexerIOSim());
                        shooter = new Shooter(new ShooterIOSim());
                        driveSimulation = new SwerveDriveSimulation(Drive.getMapleSimConfig(),
                                        new Pose2d(3.6, 6, new Rotation2d()));
                        intake = new Intake(new IntakeIOSim(driveSimulation));
                        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                        tab.add("Indexer Voltage", indexer.setVoltageCommand(0))
                                        .withWidget(BuiltInWidgets.kNumberSlider);

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
                        new Vision(drive,
                                        new VisionIOPhotonVisionSim(VisionConstants.CAMERA_FRONT_LEFT_NAME,
                                                        new Transform3d(),
                                                        driveSimulation::getSimulatedDriveTrainPose),
                                        new VisionIOPhotonVisionSim(VisionConstants.CAMERA_FRONT_RIGHT_NAME,
                                                        new Transform3d(),
                                                        driveSimulation::getSimulatedDriveTrainPose),
                                        new VisionIOPhotonVisionSim(VisionConstants.CAMERA_BACK_NAME, new Transform3d(),
                                                        driveSimulation::getSimulatedDriveTrainPose));
                        superstructure = new Superstructure(intake, indexer, shooter, leds);
                        driveInstance = drive;
                        autonomous = new Autonomous(superstructure, drive);
                }

                // makes a widget type object for networktables
                autoSide = new SendableChooser<>();
                autoSide.setDefaultOption(AutoSide.Left.autoName, AutoSide.Left);
                autoSide.addOption(AutoSide.Center.autoName, AutoSide.Center);
                autoSide.addOption(AutoSide.Right.autoName, AutoSide.Right);

                autoType = new SendableChooser<>();
                autoType.setDefaultOption(AutoType.CenterRush.autoName, AutoType.CenterRush);
                autoType.addOption(AutoType.Depot.autoName, AutoType.Depot);
                autoType.addOption(AutoType.Preload.autoName, AutoType.Preload);
                autoType.addOption(AutoType.DoubleCenter.autoName, AutoType.DoubleCenter);
                autoType.addOption(AutoType.RushDepot.autoName, AutoType.RushDepot);
                ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
                // adds the chooser object to shuffleboard
                autoTab.add("Side", autoSide);
                autoTab.add("Type", autoType);

                // targetVelocities.put(3.911, 60.0);
                // targetVelocities.put(5.18, 75.0);
                resetSimulationField();
                configurePrimaryBindings();
                configureSecondaryBindings();

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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
                JoystickButton bumperRight = new JoystickButton(joystick, (LogitechControllerButtons.bumperRight));

                // whenever drive is not being used for something else, e.g autoaligning, it
                // will do joystickDrive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -joystick.getY(),
                                                () -> -joystick.getX(),
                                                () -> -joystick.getTwist() * 0.8,

                                                () -> triggerRight.onTrue(drive.resetController())
                                                                .onFalse(drive.resetShootOnMove()).getAsBoolean()));

                bumperRight.whileTrue(superstructure.alignCommand(drive));
                bumperLeft.whileTrue(drive.alignHubPathpl());
                x.whileTrue(drive.stopWithXCommand());
                b.whileTrue(pathfinderToPose(new Pose2d(15, 7.3, new Rotation2d())));
                triggerLeft.whileTrue(DriveCommands.joystickDrive(
                                drive,
                                () -> -joystick.getY() * 0.6,
                                () -> -joystick.getX() * 0.6,
                                () -> -joystick.getTwist() * 0.6,
                                () -> triggerRight.onTrue(drive.resetController()).onFalse(drive.resetShootOnMove())
                                                .getAsBoolean()));

        }

        // dani's button bindings
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

                triggerLeft.whileTrue(Commands.parallel(shooter.setVelocityWithSlider(), indexer.setVoltageCommand(10)))
                                .onFalse(Commands.parallel(shooter.setVoltageCommand(0), indexer.setVoltageCommand(0)));

                triggerLeft.and(right).whileTrue(superstructure.oscillateShootCommand(() -> 44.0))
                                .onFalse(superstructure.storeCommand());
                triggerLeft.and(bumperRight)
                                .whileTrue(superstructure.pushShootCommand(() -> 44.0, () -> joystick.getY()));
                triggerRight
                                .whileTrue(drive.getShootOnMoveBoolean() ? superstructure.shootOnMoveCommand(drive)
                                                : superstructure.shootCommand())
                                .onFalse(superstructure.storeCommand());
                triggerRight.and(a)
                                .whileTrue(drive.getShootOnMoveBoolean() ? superstructure.shootOnMoveCommand(drive)
                                                : superstructure.shootCommand())
                                .onFalse(superstructure.storeCommand());
                triggerRight.and(right).whileTrue(superstructure.oscillateShootCommand())
                                .onFalse(superstructure.storeCommand());
                triggerRight.and(bumperRight).whileTrue(superstructure.pushShootCommand(() -> joystick.getY()));
                a.whileTrue(superstructure.intakeCommand()).onFalse(superstructure.storeCommand());
                y.whileTrue(intake.deployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
                b.whileTrue(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));
                x.whileTrue(superstructure.dumpCommand()).onFalse(superstructure.storeCommand());
                // right.whileTrue(intake.oscillateIntakeCommand());
                // up.whileTrue(indexer.setVoltageCommand(-10)).onFalse(indexer.setVoltageCommand(0));
                // down.whileTrue(intake.setSlowPositionCommand());

                up.whileTrue(intake.deployIntakeCommand(5.0));
                // down.whileTrue(intake.sysIdDynamic(Direction.kReverse)).onFalse(intake.setDeployVoltageCommand(0));

                bumperLeft.whileTrue(superstructure.passCommand()).onFalse(superstructure.storeCommand());
                bumperLeft.and(a).whileTrue(superstructure.passIntakeCommand());

        }

        public void chooseAuto() {
                boolean isLeft = autoSide.getSelected() == AutoSide.Left;
                boolean isRight = !isLeft;

                // Columbus Auto
                if (autoType.getSelected() == AutoType.CenterRush) {
                        PathPlannerAuto auto = new PathPlannerAuto("Columbus Auto", isLeft);
                        auto.activePath("Center Rush Intake Col").whileTrue(superstructure.intakeCommand())
                                        .onFalse(superstructure.storeCommand());
                        auto.activePath("Center Rush Right shoot")
                                        .onFalse(Commands.sequence(drive.autoAlignToHub().withTimeout(3),
                                                        Commands.parallel(drive.stopWithXCommand(),
                                                                        superstructure.oscillateShootCommand())));
                        autoCommand = auto;
                }

                // only do depot on the left - probably can be removed
                if (autoType.getSelected() == AutoType.Depot && isLeft) {
                        PathPlannerAuto auto = new PathPlannerAuto("Depot Auto");
                        auto.activePath("Depot Intake").whileTrue(superstructure.intakeCommand())
                                        .onFalse(superstructure.storeCommand());
                        auto.activePath("Depot to Shoot").onFalse(superstructure.shootCommand());
                        autoCommand = auto;
                }

                if (autoType.getSelected() == AutoType.DoubleCenter) {
                        PathPlannerAuto auto1 = new PathPlannerAuto("Center Rush Right", isLeft);
                        auto1
                                        .activePath("Center Rush Right Intake").whileTrue(Commands
                                                        .parallel(intake.deployIntakeCommand(
                                                                        IntakeConstants.DEPLOY_INTAKE_POSITION),
                                                                        intake.runIntakeCommand(-8)))
                                        .onFalse(intake.runIntakeCommand(0));

                        PathPlannerAuto auto2 = new PathPlannerAuto("Center Right 2 Test", isLeft);
                        auto2
                                        .activePath("Center Rush 2 1002").whileTrue(Commands
                                                        .parallel(intake.deployIntakeCommand(
                                                                        IntakeConstants.DEPLOY_INTAKE_POSITION),
                                                                        intake.runIntakeCommand(-8)))
                                        .onFalse(intake.runIntakeCommand(0));
                        autoCommand = Commands.sequence(auto1,
                                        superstructure.oscillateShootCommand().withTimeout(Seconds.of(3)),
                                        superstructure.storeCommand().withTimeout(0.1), auto2,
                                        superstructure.oscillateShootCommand());
                }

                // Dalton auto, sit still and shoot
                if (autoType.getSelected() == AutoType.Preload) {
                        superstructure.shootCommand();
                }
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                return autoCommand;

        }

        public Command pathfinderToPose(Pose2d targetPose) {
                PathConstraints constraints = new PathConstraints(4.69, 3.0, Units.degreesToRadians(540),
                                Units.degreesToRadians(720));

                Command pathfinderCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
                return pathfinderCommand;
        }

        public void resetSimulationField() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                driveSimulation.setSimulationWorldPose(new Pose2d(3.55, 6, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

        public void updateSimulation() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                SimulatedArena.getInstance().simulationPeriodic();
                Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput("FieldSimulation/Fuel",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }

        public void distanceUpdate() {
                configureSecondaryBindings();
        }

        public double getTargetVelocity() {
                return targetVelocities.get(robotDistance);
        }

        public static RobotContainer getInstance() {
                return instance;
        }

        public static Drive getDrive() {
                return driveInstance;
        }

        public Pose2d getDrivePose() {
                return drive.getPose();
        }

}
