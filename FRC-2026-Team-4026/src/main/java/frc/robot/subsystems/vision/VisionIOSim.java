package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim extends VisionIOPhotonVision{
    private static VisionSystemSim visionSim;
    private Supplier<Pose2d> robotPoseSupplier;

    private PhotonCameraSim cameraSim;

    public VisionIOSim(
        String cameraName,
        Transform3d cameraToRobot,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        super(cameraName, cameraToRobot);
        this.robotPoseSupplier = robotPoseSupplier;
        if(visionSim == null){
            visionSim = new VisionSystemSim("PhotonVision");
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera);
        cameraProperties.setFPS(50);
        visionSim.addCamera(cameraSim, cameraToRobot);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(robotPoseSupplier.get());
        super.updateInputs(inputs);
    }
    
}
