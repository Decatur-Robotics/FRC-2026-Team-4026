package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double MAX_POSE_AMBIGUITY = 0.3;

    public static final String CAMERA_FRONT_LEFT_NAME = "Front_Left_Camera_3"; // 3
    public static final String CAMERA_FRONT_RIGHT_NAME = "Front_Right_Camera_1"; // 1
        public static final String CAMERA_BACK_NAME = "Back_Camera_2"; // 2
    

         public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
        new Translation3d(Units.inchesToMeters(12),Units.inchesToMeters(12),0.24), 
        new Rotation3d(0, -0.34,Math.PI/12));
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
        new Translation3d(Units.inchesToMeters(12),-Units.inchesToMeters(12),0.24), 
        new Rotation3d(0, -0.34,-Math.PI/12));

          public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, -0.34, -Math.PI));
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
    
               public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

}
