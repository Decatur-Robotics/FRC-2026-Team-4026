package frc.robot.subsystems.vision.ColorVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Transform3d;

public class ColorVisionIOPhotonVision implements ColorVisionIO{
    protected final PhotonCamera camera;
    protected final Transform3d cameraToRobot = ColorVisionConstants.CAMERA_TO_ROBOT;
    private List<PhotonPipelineResult> objects;
    private double yaw;
 

    public ColorVisionIOPhotonVision(){

        camera = new PhotonCamera(ColorVisionConstants.CAMERA_NAME);


    }
    @Override
    public void updateInputs(ColorVisionIOInputs inputs){
        objects = camera.getAllUnreadResults();
        for (var object : objects){
            if (object.hasTargets()){
                yaw = object.getBestTarget().getYaw();
            

            }
            else{
                yaw = 0;
            }
        }


        inputs.colorVisionData = new ColorVisionIO.ColorVisionIOData(camera.isConnected(),yaw);

        

    }
}






