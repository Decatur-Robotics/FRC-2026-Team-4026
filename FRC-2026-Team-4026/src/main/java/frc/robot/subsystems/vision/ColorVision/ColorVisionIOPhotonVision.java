package frc.robot.subsystems.vision.ColorVision;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOData;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.ColorVision.ColorVisionIO.ColorVisionIOData;
import frc.robot.subsystems.vision.ColorVision.ColorVisionIO.ColorVisionIOInputs;

public class ColorVisionIOPhotonVision implements ColorVisionIO{
    protected final PhotonCamera camera;
    protected final Transform3d cameraToRobot;
    private List<PhotonPipelineResult> objects;
    private float averageYaw;
    private ArrayList<PhotonTrackedTarget> list = new ArrayList<>();
    private ArrayList<PhotonTrackedTarget> array[];


    public ColorVisionIOPhotonVision(String cameraName, Transform3d cameraToRobot){

        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;

    }
    @Override
    public void updateInputs(ColorVisionIOInputs inputs){
        objects = camera.getAllUnreadResults();
        for (var object : objects){
            if (object.hasTargets());
                for (var target:object.getTargets()){
                    if (list.isEmpty()){
                        
                        list.add(target);

                    }
                    else if((list.get(-1).getYaw() - target.getYaw()) <5){
                        list.add(target);
                    }
                    else{
                        
                    }
                }

        }


        }
        inputs.colorVisionData = new ColorVisionIOData(camera.isConnected(),averageYaw);

        


    };




}

