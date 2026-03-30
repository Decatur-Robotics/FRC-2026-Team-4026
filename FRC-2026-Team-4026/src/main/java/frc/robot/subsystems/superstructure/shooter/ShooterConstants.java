package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

    public static final double FUEL_REST_VELOCITY = 0.0;
    public static final double kP = 0.3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.15;
    public static final double kV = 0.15;
    public static final double kA = 0.015;

    public static final double SHOOTER_CURRENT_LIMIT = 60;

      public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(kP) // 0.5
        .withKI(kI)
        .withKD(kD)
        .withKS(kS) // 0.19
        .withKV(kV) // 0.13
        .withKA(kA);

    public static final double LOOK_AHEAD_SECONDS = 0.25;
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(0, 0, 0, new Rotation3d());
    public static final Angle OPTIMAL_PITCH = Radians.of(0);
    public static final Distance hubFunnelClearance = Inches.of(0);
}












