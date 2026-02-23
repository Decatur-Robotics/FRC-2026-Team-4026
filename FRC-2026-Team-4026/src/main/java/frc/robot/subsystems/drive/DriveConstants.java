package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class DriveConstants {
    public static final Distance TRACK_WIDTH = Inches.of(27.25);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(TRACK_WIDTH.in(Meters)/2, TRACK_WIDTH.in(Meters)/2), // Front Left
        new Translation2d(TRACK_WIDTH.in(Meters)/2, -TRACK_WIDTH.in(Meters)/2), // Front Right
        new Translation2d(-TRACK_WIDTH.in(Meters)/2, TRACK_WIDTH.in(Meters)/2), // Back Left
        new Translation2d(-TRACK_WIDTH.in(Meters)/2, -TRACK_WIDTH.in(Meters)/2) // Back Right
    };

    public static final double MAX_ANGULAR_VELOCITY = 10;
}
