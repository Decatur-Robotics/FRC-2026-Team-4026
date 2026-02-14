package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final double DEPLOY_INTAKE_POSITION = 0;
    public static final double STORED_INTAKE_POSITION = 0;


    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double INTAKE_VOLTAGE = 0;  
    
    public static final double DEPLOY_MOI = .5;

    public static final double ENCODER_DIST_PER_PULSE = .1757;
    public static final Distance INTAKE_WIDTH = Inches.of(20.5);
    public static final Distance INTAKE_EXTEND_LENGTH = Inches.of(8.5);
    public static final IntakeSide INTAKE_SIDE = IntakeSide.LEFT;
    public static final int INTAKE_CAPACITY = 30;

    public static final double DEPLOY_LENGTH = 9.5;
    public static final double DEPLOY_MAX_ANGLE=100;
    public static final double DEPLOY_MIN_ANGLE =0;

    public static final double INTAKING_CURRENT_THRESHOLD = 0;
}
