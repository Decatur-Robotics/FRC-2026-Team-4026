package frc.robot.subsystems.vision.ColorVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Transform3d;

public class ColorVisionIOPhotonVision implements ColorVisionIO {
    protected final PhotonCamera camera;
    private final Transform3d cameraToRobot;
    private List<PhotonPipelineResult> objects;
    private double yaw;

    public ColorVisionIOPhotonVision(String name, Transform3d cameraToRobot) {

        camera = new PhotonCamera(name);
        this.cameraToRobot = cameraToRobot;

    }
    /**
     * gets the yaw from the robot to the biggest clump of fuel
     */
    @Override
    public void updateInputs(ColorVisionIOInputs inputs) {
        objects = camera.getAllUnreadResults();

        for (var object : objects) {
            if (object.hasTargets()) {
                // gets the yaw of the biggest target and adds the cameras translation
                yaw = object.getBestTarget().getYaw() + cameraToRobot.getRotation().toRotation2d().getDegrees()
                        + cameraToRobot.getTranslation().toTranslation2d().getAngle().getDegrees();

            } else {
                yaw = 0;
            }
        }

        inputs.colorVisionData = new ColorVisionIO.ColorVisionIOData(camera.isConnected(), yaw);

    }
}
