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
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.16;
    public static final double kV = 6.0;
    public static final double kA = 1.0;

    public static final double SHOOTER_CURRENT_LIMIT = 60;

      public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.3) // 0.5
        .withKI(0)
        .withKD(0)
        .withKS(0.19) // 0.19
        .withKV(0.13) // 0.13
        .withKA(0.007);

    public static final double LOOK_AHEAD_SECONDS = 0.25;
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(0, 0, 0, new Rotation3d());
    public static final Angle OPTIMAL_PITCH = Radians.of(0);
    public static final Distance hubFunnelClearance = Inches.of(0);
}












