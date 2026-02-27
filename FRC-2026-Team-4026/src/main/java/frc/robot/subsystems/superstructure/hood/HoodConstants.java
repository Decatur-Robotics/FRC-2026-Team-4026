package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.configs.Slot0Configs;

public class HoodConstants {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.21;
    public static final double kV = 8.0;
    public static final double kA = 0;
    public static final double kG = 0.0;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
    .withKP(kP)
    .withKI(kI)
    .withKD(kD)
    .withKS(kS)
    .withKV(kV)
    .withKA(kA);


    public static final double HOOD_START_POSITION = 0.0;
}
