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

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double MAX_POSE_AMBIGUITY = 0.3;

    //     public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
    //     new Translation3d(-0.28, -0.26, 0), 
    //     new Rotation3d(0, -0.34, -Math.PI - 0.67));
    // public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
    //     new Translation3d(-0.28, 0.26, 0), 
    //     new Rotation3d(0, -0.34, -Math.PI + 0.68));
     public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
        new Translation3d(0,0,0), 
        new Rotation3d(0, 0.34,Math.PI/4));
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
        new Translation3d(0,0,0), 
        new Rotation3d(0, 0.34,-Math.PI/4));
    public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));

            public static final Matrix<N3, N1> SINGLE_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    public static final Matrix<N3, N1> MULTI_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0.25, 0.25, Double.MAX_VALUE);

    public static final String CAMERA_FRONT_LEFT_NAME = "Front_Left_Camera_3"; // 3
    public static final String CAMERA_FRONT_RIGHT_NAME = "Front_Right_Camera_1"; // 1
    public static final String CAMERA_BACK_NAME = "Back_Camera_2"; // 2

           public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

}
