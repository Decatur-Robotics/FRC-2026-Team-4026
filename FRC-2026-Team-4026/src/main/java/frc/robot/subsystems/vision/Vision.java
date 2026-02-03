package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private final VisionIO[] io;
    private final VisionConsumer consumer;
    private final VisionIOInputsAutoLogged[] inputs;
    public static double[] cameraStdevFactors = new double[]{1.0, 1.0, 1.0};

    public Vision(VisionConsumer consumer, VisionIO... io){
        this.io = io;
        this.consumer = consumer;
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for(int i = 0; i < io.length; i++){
            this.inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    public Rotation2d getTargetX(int cameraIndex){
        return inputs[cameraIndex].visionData.targetObservation().tx();
    }

    @Override
    public void periodic(){
        for(int i = 0; i < io.length; i++){
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision" + Integer.toString(i), inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++){
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for(int tagID : inputs[cameraIndex].visionData.aprilTagIDs()){
                var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagID);
                if(tagPose.isPresent()){
                    tagPoses.add(tagPose.get());
                }
        }
    

        for(var observation: inputs[cameraIndex].visionData.poseObservation()){
            boolean rejectPose = observation.tagCount() == 0
            || (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_POSE_AMBIGUITY)
            || Math.abs(observation.pose().getZ()) > 0.75
            //Field boundaries
            ||observation.pose().getX() < 0.0
            ||observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
            ||observation.pose().getY() < 0.0
            ||observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth() 
            ;

            robotPoses.add(observation.pose());
            if(rejectPose){
                robotPosesRejected.add(observation.pose());
            }
            else{
                robotPosesAccepted.add(observation.pose());
            }

            if(rejectPose){
                continue;
            }

            double stDevFactor =  Math.pow(observation.averageTagDistance(), 2.0)/observation.tagCount();
            double linearStDev = .02 * stDevFactor;
            double angularStDev = .05 * stDevFactor;

            if (observation.poseType() == VisionIO.PoseObservationType.MEGATAG_2) {
                linearStDev *= .5;
                angularStDev *= 0;
            }

            consumer.accept(observation.pose().toPose2d(),
             cameraIndex, 
             VecBuilder.fill(linearStDev, linearStDev, angularStDev));
        }
        // Log camera data
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
    }
    }

        @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
