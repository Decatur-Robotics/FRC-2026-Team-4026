package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {

    public static final double FUEL_REST_VELOCITY = 0.0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.16;
    public static final double kV = 6.0;
    public static final double kA = 1.0;

      public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.5) // 0.5
        .withKI(0)
        .withKD(0)
        .withKS(0.19) // 0.19
        .withKV(0.13) // 0.13
        .withKA(0.007);

}
