package frc.robot.subsystems.superstructure;

import frc.robot.util.SuperstructureState;

public class SuperstructureConstants {
    public static final SuperstructureState STARTING_STATE = new SuperstructureState(
        0.0,    // shooterVelocity
        0.0,    // hoodAngle
        0.0,    // intakeDeployed
        0.0,    // indexerVoltage
        0.0     // intakeVoltage
    );

    public static final SuperstructureState INTAKE_STATE = new SuperstructureState(
        0.0,    // shooterVelocity
        0.0,    // hoodAngle
        1.0,    // intakeDeployed
        0.0,    // indexerVoltage
        12.0    // intakeVoltage
    );

    
}
