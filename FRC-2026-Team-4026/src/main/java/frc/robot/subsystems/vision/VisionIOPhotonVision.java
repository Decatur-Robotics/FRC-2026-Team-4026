package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;

    public VisionIOPhotonVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }
}
