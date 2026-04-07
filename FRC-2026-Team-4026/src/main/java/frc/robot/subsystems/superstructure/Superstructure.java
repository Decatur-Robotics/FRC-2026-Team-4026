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

    //This is for making out robot both harder to defend and makes the chance of robot damage lower
    private boolean defenseMode = false;


    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Leds leds) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;

        this.leds = leds;


        this.targetState = SuperstructureConstants.STARTING_STATE;
    }

    public Command setState(SuperstructureState targetState) {

        this.targetState = targetState.copyInstatnce();
        return Commands.parallel(
                shooter.setVelocityCommand(targetState.shooterVelocity),
                intake.deployIntakeCommand(targetState.intakeDeployed),
                intake.runIntakeCommand(targetState.intakeVoltage),
                indexer.setVoltageCommand(targetState.indexerVoltage));
    }

    @Override
    public void periodic() {
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
        return new SuperstructureState(shooter.getVelocity(), intake.getDeployPosition(), indexer.getMecanumVoltage(), intake.getIntakeVoltage());
    }

    // public boolean isHoodAtTarget(){
    //     return (hood.getPosition() < targetState.hoodAngle + SuperstructureConstants.HOOD_DEADBAND) && (hood.getPosition() > targetState.hoodAngle - SuperstructureConstants.HOOD_DEADBAND);
    // }


    public Command startingCommand() {
        return Commands.parallel(setState(SuperstructureConstants.STARTING_STATE));
    }

    public Command intakeCommand() {
        return Commands.parallel(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-8), shooter.setVoltageCommand(0), leds.pulsingCommand());
    }

    public Command altIntakeCommand() {
        return Commands.parallel(intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-12), shooter.setVoltageCommand(getNumBallsStored()));
    }

    public Command testShootCommand() {
        return Commands.parallel(shooter.setVelocityCommand(55), indexer.setVoltageCommand(10));
    }

    public Command noTestShootCommands() {
        return Commands.parallel(shooter.setVelocityCommand(0), indexer.setVoltageCommand(0));
    }

    public Command storeCommand() {
        return Commands.parallel(setState(SuperstructureConstants.STORING_STATE), shooter.setVoltageCommand(0), intake.runIntakeCommand(0));
    }

    public Command dumpCommand() {
        return Commands.parallel(intake.runIntakeCommand(6), indexer.setVoltageCommand(6));
    }

    // public Command shootCommand(){
    //     // if(!Robot.isReal()){

    //     // }
    //     // if(defenseMode && RobotState.CURRENT_LIMITS_EXCEEDED || defenseMode && RobotState.BATTERY_BROWNOUT_PROTECTION){
    //     //     return Commands.parallel(setState(new SuperstructureState(0.0,0.0,0.0,0.0,0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
    //     // }
    //     // else{
    //     //     // if (RobotState.BATTERY_BROWNOUT_PROTECTION){
    //     //         return Commands.parallel(setState( new SuperstructureState(robotState.getTargetVelocity() < SuperstructureConstants.VELOCITY_BROWNOUT_LIMIT ? robotState.getTargetVelocity() : 0.0,  
    //     //         robotState.getTargetAim(), 1.0, 6, 6)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 10));

    //     //     // }
    //     //    }


    //     // else {
    //     //     return Commands.parallel(setState(new SuperstructureState(Math.min(robotState.getTargetVelocity(), robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())),robotState.getTargetAim(),0.0,12*robotState.getBrownoutVoltage(),0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
    //     // }}
    //     if(getDefenseMode()){
    //         return Commands.parallel(shootCommand(ShotEstimator.getInstance(drive).getTargetVelocity()), drive.setRotationXCommand());
    //     } else {
    //         return shootCommand(ShotEstimator.getInstance(drive).getTargetVelocity());
    //     }
    // }

    public Command shootCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.shootAimCommand(), leds.shooterWindUpCommand(shooter.getVelocity(), ShotEstimator.getInstance().getTargetVelocity().get())).
                        until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get() - 2),
                Commands.parallel(shooter.shootAimCommand(), indexer.setVoltageCommand(10), leds.rainbowCommand(), intake.runIntakeCommand(-0.5))
        );
    }


    public Command shootCommand(Supplier<Double> velocity) {
        return Commands.sequence(shooter.setVelocityCommand(velocity.get()), Commands.waitUntil(() -> shooter.getVelocity() > (velocity.get() - 2.0)), Commands.parallel(shooter.setVelocityCommand(velocity.get()), indexer.setVoltageCommand(10), intake.runIntakeCommand(-0.5)));
    }


    public Command passCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.passAimCommand(), leds.shooterWindUpCommand(shooter.getVelocity(), ShotEstimator.getInstance().getTargetVelocity().get())).
                        until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get() - 0.5),
                Commands.parallel(shooter.passAimCommand(), indexer.setVoltageCommand(10))
        );
    }

    public Command passIntakeCommand() {
        return Commands.parallel(passCommand(), intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-12));
    }

    public Command testShootCommands() {
        return Commands.parallel(shooter.setVelocityCommand(45), indexer.setVoltageCommand(12));
    }

    public Command stopTestShootCommand() {
        return Commands.parallel(shooter.setVelocityCommand(0), indexer.setVoltageCommand(0));
    }

    public Command shootAndIntake() {
        return Commands.parallel(shootCommand(), intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-10));
    }

    public Command shootAndIntakeOnMove(Drive drive) {
        return Commands.parallel(shootOnMoveCommand(drive), intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION), intake.runIntakeCommand(-10));
    }


    public Command oscillateShootCommand() {
        return Commands.sequence(
                Commands.parallel(shooter.shootAimCommand(), leds.shooterWindUpCommand(shooter.getVelocity(), ShotEstimator.getInstance().getTargetVelocity().get()))
                        .until(() -> shooter.getVelocity() > ShotEstimator.getInstance().getTargetVelocity().get() - 2),
                Commands.parallel(shooter.shootAimCommand(), indexer.setVoltageCommand(10), leds.rainbowCommand(), intake.oscillateIntakeCommand())
        );
    }

    public Command oscillateShootCommand(Supplier<Double> velocity) {
        return Commands.parallel(shootCommand(velocity), intake.oscillateIntakeCommand());
    }

    public Command pushShootCommand(Supplier<Double> deployPosition) {
        return Commands.parallel(shootCommand(), intake.deployIntakeCommand(-deployPosition.get() * 15.3 + 15.3));
    }

    public Command pushShootCommand(Supplier<Double> velocity, Supplier<Double> deployPosition) {
        return Commands.parallel(shootCommand(velocity), intake.deployIntakeCommand(-deployPosition.get() * 15.3 + 15.3));
    }

    public Command shootOnMoveCommand(Drive drive) {
        return Commands.parallel(shooter.shootOnMoveCommand(drive), indexer.setVoltageCommand(10), intake.runIntakeCommand(-2));
    }

    public Command testingShootCommand() {
        return setState(new SuperstructureState(1, 0.0, 0, 12, 0.0));
    }

    public Command testShootAutoCommand() {
        return Commands.parallel(indexer.setVoltageCommand(8), shooter.setVelocityCommand(45), intake.runIntakeCommand(-4));
    }


    public Command noTestShootCommand() {
        return Commands.parallel(indexer.setVoltageCommand(0), shooter.setVoltageCommand(0));
    }
    // public Command passCommand(){
    // //    if (defenseMode) {
    //         // if (robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())<robotState.getTargetVelocity()+10){
    //         //     return Commands.parallel(setState(new SuperstructureState(0.0, 0.0, 0.0, 0.0, 0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
    //         // }
    //         // else {
    //             // return Commands.parallel(setState(new SuperstructureState(Math.min(robotState.getTargetVelocity()+10, robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())),robotState.getTargetAim()+0.1,0.0,0.0,0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
    //         // }
    // //    }
    // //    else {
    //     //  return setState(new SuperstructureState(ShotEstimator.getInstance(drive).getTargetVelocity().get() + 10, 0, 8.0));
    //     // }
    //     }


    public Command retractIntakeCommand() {
        return Commands.runOnce(() -> intake.deployIntakeCommand(IntakeConstants.STORED_INTAKE_POSITION));
    }

    public Command deployIntakeCommand() {
        return Commands.runOnce(() -> intake.deployIntakeCommand(IntakeConstants.DEPLOY_INTAKE_POSITION));
    }

    public int getNumBallsStored() {
        return intake.getNumBallsIntaked() - shooter.getNumBallsShot();
    }

    public Command alignCommand(Drive drive) {
        // return Commands.parallel(drive.autoAlignToHub(),drive.isAligned()?leds.correctCommand():leds.aligningCommand());
        return Commands.parallel(drive.autoAlignToHub(), Commands.sequence(leds.aligningCommand(), Commands.waitUntil(() -> drive.isAligned()), leds.correctCommand()));

    }


}