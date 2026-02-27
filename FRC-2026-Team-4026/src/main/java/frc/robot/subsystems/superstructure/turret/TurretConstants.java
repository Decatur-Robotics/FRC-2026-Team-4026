package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class TurretConstants {
    public static final double kP = 0.0;
    public static final double kI = 0.0;   
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final Matrix<N3, N1> TRAJECTORY_WEIGHTS = new Matrix<>(Nat.N3(), Nat.N1());

    public static final double TURRET_STARTING_POSITION = 0.0;
}
