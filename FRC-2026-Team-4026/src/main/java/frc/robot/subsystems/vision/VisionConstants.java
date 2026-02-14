package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double MAX_POSE_AMBIGUITY = 0.3;

        public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, -0, 0));
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));
    public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));

    public static final String CAMERA_FRONT_LEFT_NAME = "Front Left Camera 3"; // 3
    public static final String CAMERA_FRONT_RIGHT_NAME = "Front Right Camera 1"; // 1
    public static final String CAMERA_BACK_NAME = "Back Camera 2"; // 2

}
