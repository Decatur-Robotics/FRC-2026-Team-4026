package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.leds.Leds;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShotEstimator;
import frc.robot.util.SuperstructureState;

public class Superstructure extends SubsystemBase {
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;

    private frc.robot.subsystems.superstructure.leds.Leds leds;

    private boolean isSimulation = Robot.isSimulation();

    private SuperstructureState targetState;

    // This is for making out robot both harder to defend and makes the chance of
    // robot damage lower
    private boolean defenseMode = false;

    /**
     * Contains all of the command for the main mechanisms
     * @param intake the intake object
     * @param indexer the indexer object
     * @param shooter the shooter object
     * @param leds the LED object
     */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Leds leds) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;

        this.leds = leds;

        this.targetState = SuperstructureConstants.STARTING_STATE;
    }

    /**
     * Sets the shooter velocity, intake position and voltage, and the indexer voltage
     * @param targetState the superstructure state which contains the target values for the mechanisms
     * @return the command running all the mechanisms
     */
    public Command setState(SuperstructureState targetState) {

        this.targetState = targetState.copyInstatnce();
        return Commands.parallel(
                shooter.setVelocityCommand(targetState.shooterVelocity),
                intake.deployIntakeCommand(targetState.intakeDeployed),
                intake.runIntakeCommand(targetState.intakeVoltage),
                indexer.setVoltageCommand(targetState.indexerVoltage));
    }

    public void toggleDefenseMode() {
        if (defenseMode) {
            defenseMode = false;
        } else {
            defenseMode = true;
        }
    }

    public Command toggleDefenseModeCommand() {
        return Commands.run(() -> toggleDefenseMode());
    }

    public boolean getDefenseMode() {
        return defenseMode;
    }

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(shooter.getVelocity(), intake.getDeployPosition(), indexer.getMecanumVoltage(),
                intake.getIntakeVoltage());
    }

    public Command startingCommand() {
        return Commands.parallel(setState(SuperstructureConstants.STARTING_STATE));
    }

    public Command intakeCommand() {
        return Commands.parallel(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION),
                intake.runIntakeCommand(-10), shooter.setVoltageCommand(0), leds.pulsingCommand());
    }

    public Command testShootCommand() {
        return Commands.parallel(shooter.setVelocityCommand(55), indexer.setVoltageCommand(10));
    }

    public Command noTestShootCommands() {
        return Commands.parallel(shooter.setVelocityCommand(0), indexer.setVoltageCommand(0));
    }

    /**
     * has the intake deployed and all mechanism not running to store balls
     */
    public Command storeCommand() {
        return Commands.parallel(setState(SuperstructureConstants.STORING_STATE), shooter.setVoltageCommand(0),
                intake.runIntakeCommand(0));
    }

    public Command dumpCommand() {
        return Commands.parallel(intake.runIntakeCommand(10), indexer.setVoltageCommand(-10));
    }

    /**
     * Shoots balls and indexes when the shooter is up to velocity
     */
    public Command shootCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.shootAimCommand(),
                        leds.shooterWindUpCommand(shooter.getVelocity(),
                                ShotEstimator.getInstance().getTargetVelocity().get()))
                        .until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get()
                                - 0.5),
                Commands.parallel(shooter.shootAimCommand(), indexer.setVoltageCommand(10), leds.rainbowCommand(),
                        intake.runIntakeCommand(-0.5)));
    }

    public Command shootCommand(Supplier<Double> velocity) {
        return Commands.sequence(shooter.setVelocityCommand(velocity.get()),
                Commands.waitUntil(() -> shooter.getVelocity() > (velocity.get() - 0.5)),
                Commands.parallel(shooter.setVelocityCommand(velocity.get()), indexer.setVoltageCommand(10),
                        intake.runIntakeCommand(-0.5)));
    }

    public Command passCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.passAimCommand(),
                        leds.shooterWindUpCommand(shooter.getVelocity(),
                                ShotEstimator.getInstance().getTargetVelocity().get()))
                        .until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get()
                                - 0.5),
                Commands.parallel(shooter.passAimCommand(), indexer.setVoltageCommand(10)));
    }

    public Command passIntakeCommand() {
        return Commands.parallel(passCommand(), intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION),
                intake.runIntakeCommand(-10));
    }

    public Command testShootCommands() {
        return Commands.parallel(shooter.setVelocityCommand(45), indexer.setVoltageCommand(12));
    }

    public Command stopTestShootCommand() {
        return Commands.parallel(shooter.setVelocityCommand(0), indexer.setVoltageCommand(0));
    }

    public Command shootAndIntake() {
        return Commands.parallel(shootCommand(), intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION),
                intake.runIntakeCommand(-10));
    }

    public Command shootAndIntakeOnMove(Drive drive) {
        return Commands.parallel(shootOnMoveCommand(drive),
                intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-10));
    }

    /**
     * Shoots and aims while also oscillating the intake for improved indexing
     * @return the command running the oscillate shoot command
     */
    public Command oscillateShootCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.shootAimCommand(),
                        leds.shooterWindUpCommand(shooter.getVelocity(),
                                ShotEstimator.getInstance().getTargetVelocity().get()))
                        .until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get() - 2),
                Commands.parallel(shooter.shootAimCommand(), indexer.setVoltageCommand(10), leds.rainbowCommand(),
                        intake.oscillateIntakeCommand()));
    }

    /**
     * runs the oscillate shoot command put with a preset velocity
     * @param velocity the target velocity of the shooter
     * @return the oscillate shoot command
     */
    public Command oscillateShootCommand(Supplier<Double> velocity) {
        return Commands.parallel(shootCommand(velocity), intake.oscillateIntakeCommand());
    }

    /**
     * pushes up the intake while shooting
     * @param deployPosition target end position of the intake
     * @return the command to run the mechanisms
     */
    public Command pushShootCommand(Supplier<Double> deployPosition) {
        return Commands.parallel(shootCommand(), intake.deployIntakeCommand(-deployPosition.get() * 15.3 + 15.3));
    }

    public Command pushShootCommand(Supplier<Double> velocity, Supplier<Double> deployPosition) {
        return Commands.parallel(shootCommand(velocity),
                intake.deployIntakeCommand(-deployPosition.get() * 15.3 + 15.3));
    }

    /**
     * sets the shooter velocity based on the drive velocity
     * @param drive inputs the drivetrain
     * @return the full mechanism command
     */
    public Command shootOnMoveCommand(Drive drive) {
        return Commands.parallel(shooter.shootOnMoveCommand(drive), indexer.setVoltageCommand(10),
                intake.runIntakeCommand(-0.5));
    }

    public Command testingShootCommand() {
        return setState(new SuperstructureState(1, 0.0, 0, 12, 0.0));
    }

    public Command noTestShootCommand() {
        return Commands.parallel(indexer.setVoltageCommand(0), shooter.setVoltageCommand(0));
    }

    /**
     * sets the intake position to its inner position
     */
    public Command retractIntakeCommand() {
        return Commands.runOnce(() -> intake.deployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
    }

        /**
     * sets the intake position to its outer position
     */
    public Command deployIntakeCommand() {
        return Commands.runOnce(() -> intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));
    }

    public Command alignCommand(Drive drive) {
        return Commands.parallel(drive.autoAlignToHub(), Commands.sequence(leds.aligningCommand(),
                Commands.waitUntil(() -> drive.isAligned()), leds.correctCommand()));
    }

}