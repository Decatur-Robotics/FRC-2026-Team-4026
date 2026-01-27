package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private Pose2d robotPose = new Pose2d(0,0, new Rotation2d());
  private double turretRotation = Math.atan((robotPose.getY() - FieldConstants.HUB_POSE_BLUE.getY())/(robotPose.getX() - FieldConstants.HUB_POSE_BLUE.getX()));

    private SuperstructureState targetState;

    private double shooterVelocity;
    private double hoodAngle;

    

    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Hood hood, leds leds) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.hood = hood;
        this.leds = leds;
        

        this.shooterVelocity = 0.0;
        this.hoodAngle = 0.0;
        
        leds.setAllLedsCommand(ledsConstants.BLUE);

        this.targetState = null;


    }

    public Command setState(){

        this.targetState = targetState.copyInstatnce();
        return Commands.parallel(shooter.setVelocityCommand(targetState.shooterVelocity),
        hood.setPositionCommand(targetState.hoodAngle),
        intake.deployIntakeCommand(targetState.intakeDeployed),
        intake.runIntakeCommand(targetState.intakeVoltage),
        indexer.setVoltageCommand(targetState.indexerVoltage));
    }


}
