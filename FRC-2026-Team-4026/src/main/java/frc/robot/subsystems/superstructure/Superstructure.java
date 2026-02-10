package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.util.SuperstructureState;
import frc.robot.subsystems.superstructure.leds.leds;
import frc.robot.subsystems.superstructure.leds.ledsConstants;

public class Superstructure extends SubsystemBase {
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private Hood hood;
    private leds leds;
    private RobotState robotState;

  //private double turretRotation = Math.atan((robotPose.getY() - FieldConstants.HUB_POSE_BLUE.getY())/(robotPose.getX() - FieldConstants.HUB_POSE_BLUE.getX()));

    private boolean isSimulation = Robot.isSimulation();

    private SuperstructureState targetState;

    private double shooterVelocity;
    private double hoodAngle;

    //This is for making out robot both harder to defend and makes the chance of robot damage lower
    private boolean defenseMode = false;
    

    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Hood hood, leds leds, RobotState robotState) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.hood = hood;
        this.leds = leds;

        this.robotState = robotState;
        

        this.shooterVelocity = 0.0;
        this.hoodAngle = 0.0;
        
        leds.setAllLedsCommand(ledsConstants.BLUE);

        this.targetState = SuperstructureConstants.STARTING_STATE;

        // setDefaultCommand(storeCommand());

    }

    public Command setState(SuperstructureState targetState){

        this.targetState = targetState.copyInstatnce();
        return Commands.parallel(shooter.setVelocityCommand(targetState.shooterVelocity),
        hood.setPositionCommand(targetState.hoodAngle),
        intake.deployIntakeCommand(targetState.intakeDeployed),
        intake.runIntakeCommand(targetState.intakeVoltage),
        indexer.setVoltageCommand(targetState.indexerVoltage));
    }

    public void toggleDefenseMode(){
        if(defenseMode){
            defenseMode = false;
        } else {
            defenseMode = true;
        }
    }

    public SuperstructureState getCurrentState(){
        return new SuperstructureState(shooter.getVelocity(), hood.getPosition(), intake.getDeployPosition(), indexer.getMecanumVoltage(), intake.getIntakeVoltage());
    }

    public boolean isHoodAtTarget(){
        return (hood.getPosition() < targetState.hoodAngle + SuperstructureConstants.HOOD_DEADBAND) && (hood.getPosition() > targetState.hoodAngle - SuperstructureConstants.HOOD_DEADBAND);
    }


    public Command startingCommand(){
        return Commands.parallel(setState(SuperstructureConstants.STARTING_STATE), leds.setAllLedsCommand(ledsConstants.BLUE));
    }

    public Command intakeCommand(){
        if (defenseMode){
            return Commands.parallel(setState(new SuperstructureState(0.0, 0.0, 1.0, 0.0, 12*robotState.getBrownoutVoltage())), leds.pulseLedsCommand(ledsConstants.GREEN, 5));
        }
        return Commands.parallel(setState(SuperstructureConstants.INTAKE_STATE), leds.pulseLedsCommand(ledsConstants.GREEN, 5));
    }

    public Command storeCommand(){
        if(defenseMode){
            return Commands.parallel(setState(new SuperstructureState(0.0, 0.0, 0.0, 0.0, 0.0)), leds.setAllLedsCommand(ledsConstants.BLUE));
        }

        else{
            return Commands.parallel(setState(SuperstructureConstants.STORING_STATE), leds.setAllLedsCommand(ledsConstants.BLUE));
        }
    }

    public Command dumpCommand(){
        return Commands.parallel(setState(SuperstructureConstants.DUMPING_STATE), leds.pulseLedsCommand(ledsConstants.RED, 5));
    }

    public Command shootCommand(){
        if(defenseMode && RobotState.CURRENT_LIMITS_EXCEEDED || defenseMode && RobotState.BATTERY_BROWNOUT_PROTECTION){
            return Commands.parallel(setState(new SuperstructureState(0.0,0.0,0.0,0.0,0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
        }
        else{
            if (RobotState.BATTERY_BROWNOUT_PROTECTION){
                return Commands.parallel(setState( new SuperstructureState(robotState.getTargetVelocity() < SuperstructureConstants.VELOCITY_BROWNOUT_LIMIT ? robotState.getTargetVelocity() : 0.0,  
                robotState.getTargetAim(), 1.0, 6, 6)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 10));
                
            }
        else {
            return Commands.parallel(setState(new SuperstructureState(Math.min(robotState.getTargetVelocity(), robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())),robotState.getTargetAim(),0.0,12*robotState.getBrownoutVoltage(),0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));

    public Command passCommand(){
        if(defenseMode && RobotState.CURRENT_LIMITS_EXCEEDED || defenseMode && RobotState.BATTERY_BROWNOUT_PROTECTION){
            return Commands.parallel(setState(new SuperstructureState(0.0,0.0,0.0,0.0,0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
        }
            
        }
        else {
            return Commands.parallel(setState(new SuperstructureState(robotState.getTargetVelocity(), robotState.getTargetAim(), 0.0, 12.0, 0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
        }
        }
        

    public Command passCommand(){
       if (defenseMode) {
            if (robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())<robotState.getTargetVelocity()+10){
                return Commands.parallel(setState(new SuperstructureState(0.0, 0.0, 0.0, 0.0, 0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
            }
            else {
                return Commands.parallel(setState(new SuperstructureState(Math.min(robotState.getTargetVelocity()+10, robotState.getVoltageToVelocity(robotState.getBrownoutVoltage())),robotState.getTargetAim()+0.1,0.0,0.0,0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));

            }
       }
       else {
            return Commands.parallel(setState(new SuperstructureState(robotState.getTargetVelocity()+10, robotState.getTargetAim()+0.1, 0.0, 12.0, 0.0)), leds.flashAllLedsCommand(ledsConstants.YELLOW, 0));
        }
        }
            
    

    public void shootFuel(){

        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
            robotState.getDrivePose(),
            new Translation2d(0,0),
            robotState.getChassisSpeed(),
            robotState.getDriveRotatoin(),
            Meters.of(0.2),
            MetersPerSecond.of(2),
            Radians.of(hood.getPosition())
        );

        fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
}

 public Command shootFuelCommand(){
        return Commands.runOnce(()->shootFuel());
 }



}

 public Command shootFuelCommand(){
        return Commands.runOnce(()->shootFuel());
 }

}