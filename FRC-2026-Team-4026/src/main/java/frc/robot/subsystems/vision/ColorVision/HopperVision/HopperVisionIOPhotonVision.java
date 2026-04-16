package frc.robot.subsystems.vision.ColorVision.HopperVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;

public class HopperVisionIOPhotonVision implements HopperVisionIO {
    protected final PhotonCamera camera;

    private List<PhotonPipelineResult> objects;
    private double ballsInHopper;
    private double areaOfHopper;
    private double percentAreaOfHopperFilled;

    public HopperVisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);

    }

    @Override
    public void updateInputs(HopperVisionIOInputs inputs) {
        objects = camera.getAllUnreadResults();
        for (var object : objects) {
            if (object.hasTargets()) {
                // gets the percentage of the screen filled by the bounding box
                areaOfHopper = object.getBestTarget().getArea();
                percentAreaOfHopperFilled = (HopperVisionConstants.AREA_OF_FULL_HOPPER / areaOfHopper);
                ballsInHopper = percentAreaOfHopperFilled * HopperVisionConstants.MAX_BALLS_IN_HOPPER;
            } else {
                areaOfHopper = 0;
                ballsInHopper = 0;
                percentAreaOfHopperFilled = 0;
            }
        }

        inputs.hopperVisionData = new HopperVisionIO.HopperVisionIOData(camera.isConnected(), ballsInHopper,
                percentAreaOfHopperFilled);

    }
}
