package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.intake.IntakeConstants;
import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {
    public static final double SHOOTING_TRENCH = 40.0;
    public static final double HOOD_DEADBAND = 0.0;
    //velocity brownout limit needs to be increased
    public static final double VELOCITY_BROWNOUT_LIMIT = 0.0;
    public static final SuperstructureState STARTING_STATE = new SuperstructureState(
        0.0,    // shooterVelocity
        0.0,    // hoodAngle
        0.0,    // intakeDeployed
        0.0,    // indexerVoltage
        0.0     // intakeVoltage
    );

    public static final SuperstructureState INTAKE_STATE = new SuperstructureState(
        IntakeConstants.DEPLOY_INTAKE_POSITION,    // intakeDeployed
        -10.0    // intakeVoltage
    );

    public static final SuperstructureState STORING_STATE = new SuperstructureState(
        IntakeConstants.NORMAL_INTAKE_POSITION,  // intakeDeployed
        0.0   // intakeVoltage
    );

    public static final SuperstructureState CONTAINING_STATE = new SuperstructureState(IntakeConstants.STORED_INTAKE_POSITION, 0);

    public static final SuperstructureState DUMPING_STATE = new SuperstructureState(
        6.0,  // indexerVoltage
        -6.0     // intakeVoltage
    );



    
}
