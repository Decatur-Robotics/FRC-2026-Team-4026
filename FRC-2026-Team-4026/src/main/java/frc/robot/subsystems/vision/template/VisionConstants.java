package frc.robot.subsystems.vision.template;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
     public static final String CAMERA_FRONT_LEFT_NAME = "Front_Left_Camera_3"; // 3
    public static final String CAMERA_FRONT_RIGHT_NAME = "Front_Right_Camera_1"; // 1

         public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
        new Translation3d(0,0,0), 
        new Rotation3d(0, -0.34,Math.PI/4));
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
        new Translation3d(0,0,0), 
        new Rotation3d(0, -0.34,-Math.PI/4));
      // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;  
}
