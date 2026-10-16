package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public static final double STORED_INTAKE_POSITION = 1.0;
    public static final double DEPLOY_INTAKE_POSITION = 15.3;
    public static final double NORMAL_INTAKE_POSITION = 14;

    public static final double OSCILLATING_TIMER = 0.5;

    public static final double kP = 0.08;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.4;
    public static final double kV = 0.1;
    public static final double kA = 0.07;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
    .withKP(kP)
    .withKI(kI)
    .withKD(kD)
    .withKS(kS)
    .withKV(kV)
    .withKA(kA);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
    .withMotionMagicAcceleration(20)
    .withMotionMagicCruiseVelocity(7.5);

    public static final double INTAKE_VOLTAGE = -8;  
    
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
    public static final double DEPLOY_CURRENT_LIMIT = 45;
    public static final double HALFWAY_INTAKE_POSITION = 7;
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
    .withFeedbackRemoteSensorID(0)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withSensorToMechanismRatio(1)
    .withRotorToSensorRatio(47);//please add real gear ratio pretty please love you
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

}
