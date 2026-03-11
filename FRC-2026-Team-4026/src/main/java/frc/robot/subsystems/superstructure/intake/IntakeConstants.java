package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final double STORED_INTAKE_POSITION = 1.0;
    public static final double DEPLOY_INTAKE_POSITION = 15.3;//15.3

    public static final double kP = 0.08;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.25;
    public static final double kV = 8.0;
    public static final double kA = 6.0;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
    .withKP(kP)
    .withKI(kI)
    .withKD(kD)
    .withKS(kS)
    .withKV(kV)
    .withKA(kA);

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

    public static final double INTAKE_CURRENT_LIMIT = 70;
    public static final double DEPLOY_CURRENT_LIMIT = 60;


}
