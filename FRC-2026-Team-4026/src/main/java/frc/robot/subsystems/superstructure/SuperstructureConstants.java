package frc.robot.subsystems.superstructure;

import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {

    //velocity brownout limit needs to be increased
    public static final double VELOCITY_BROWNOUT_LIMIT = 0.0;
    public static final SuperstructureState STARTING_STATE = new SuperstructureState(
        0.0,    // shooterVelocity
        0.0,    // intakeDeployed
        0.0,    // indexerVoltage
        0.0     // intakeVoltage
    );
    // I messed with these commands you can change it back
    public static final SuperstructureState INTAKE_STATE = new SuperstructureState(

    15.3,
    8);

    public static final SuperstructureState STORING_STATE = new SuperstructureState(
    0,
    0);
    
}
