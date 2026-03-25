package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.subsystems.drive.Drive;

public class ColorVision {
    private ColorVisionIO io;
    private final ColorVisionIOInputsAutoLogged inputs = new ColorVisionIOInputsAutoLogged();
    private double yaw;

    public ColorVision(ColorVisionIO io){
        this.io = io;
        yaw = inputs.colorVisionData.yaw();

    }
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("ColorVision", inputs);
        Logger.recordOutput("yaw",inputs.colorVisionData.yaw());

    }
    public Rotation2d getRotationChange(Drive drive){
        yaw = inputs.colorVisionData.yaw();
        if (inputs.colorVisionData.yaw() == 0){

            return Rotation2d.fromDegrees(drive.getRotation().getDegrees()+45);
        }
        else{
            return Rotation2d.fromDegrees(inputs.colorVisionData.yaw()+ drive.getRotation().getDegrees());
        }
    }
}
