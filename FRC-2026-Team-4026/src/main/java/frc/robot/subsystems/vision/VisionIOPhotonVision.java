package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d cameraToRobot;
    protected TargetObservation targetObservation;

    public VisionIOPhotonVision(String cameraName, Transform3d cameraToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Set<Short> aprilTagIDs = new HashSet<>();
        List<PoseObservation> poseObservations = new ArrayList<>();
        for(var result : camera.getAllUnreadResults()){
            if(result.hasTargets()){
                targetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch())
                );
            }
            else{
                targetObservation = new TargetObservation(
                    new Rotation2d(),
                    new Rotation2d()
                );
            }

            if(result.multitagResult.isPresent()){
                var multiTagResult = result.multitagResult.get();
                
                Transform3d fieldToCamera = multiTagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(cameraToRobot);

                Pose3d robotPose = new Pose3d(
                    fieldToRobot.getTranslation(),
                    fieldToRobot.getRotation()
                );

                double totalTagDistance = 0.0;
                for(var target : result.getTargets()){
                    totalTagDistance += 
                        target.getBestCameraToTarget().getTranslation().getNorm();
                }

                aprilTagIDs.addAll(multiTagResult.fiducialIDsUsed);

                poseObservations.add(
                    new PoseObservation(result.getTimestampSeconds(),
                    robotPose,
                    multiTagResult.estimatedPose.ambiguity,
                    multiTagResult.fiducialIDsUsed.size(),
                    totalTagDistance/result.targets.size(),
                    PoseObservationType.PHOTONVISION)
                );
            }
            else if(!result.targets.isEmpty()){
                var target = result.targets.get(0);

                var tagPose = VisionConstants.aprilTagLayout.getTagPose(target.fiducialId);
                if(tagPose.isPresent()){
                    Transform3d tagToCamera = target.getBestCameraToTarget();
                    Transform3d fieldToTag = new Transform3d(
                        tagPose.get().getTranslation(),
                        tagPose.get().getRotation()
                    );
                    Transform3d cameraToTag = tagToCamera.inverse();
                    Transform3d fieldToCamera = fieldToTag.plus(cameraToTag);
                    Transform3d fieldToRobot = fieldToCamera.plus(cameraToRobot.inverse());

                    Pose3d robotPose = new Pose3d(
                        fieldToRobot.getTranslation(),
                        fieldToRobot.getRotation()
                    );

                    aprilTagIDs.add((short)target.fiducialId);

                    poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        target.getPoseAmbiguity(),
                        1,
                        cameraToTag.getTranslation().getNorm(),
                        PoseObservationType.PHOTONVISION
                    ));
                }
            }
        }

        PoseObservation[] poseObservationArray = new PoseObservation[poseObservations.size()];
        for(int i = 0; i < poseObservations.size(); i++){
            poseObservationArray[i] = poseObservations.get(i);
        }

        int[] aprilTagIDArray = new int[aprilTagIDs.size()];
        int i = 0;
        for(var id : aprilTagIDs){
            aprilTagIDArray[i++] = id;
            }

        // inputs.visionData = new VisionIOData(
        //     camera.isConnected(),
        //     targetObservation,
        //     poseObservationArray,
        //     aprilTagIDArray
        // );

        }
}
