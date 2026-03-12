package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.subsystems.drive.Drive;

public class ColorVision {
    private ColorVisionIO io;
    private final ColorVisionIOInputsAutoLogged inputs = new ColorVisionIOInputsAutoLogged();
    private double averageYaw;

    public ColorVision(ColorVisionIO io){
        this.io = io;
        averageYaw = inputs.colorVisionData.averageYaw();

    }
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("ColorVision", inputs);
        Logger.recordOutput("averageYaw",inputs.colorVisionData);

    }
    public Rotation2d getRotationChange(Drive drive){
        averageYaw = inputs.colorVisionData.averageYaw();
        if (inputs.colorVisionData.averageYaw() == 0){

            return Rotation2d.fromDegrees(drive.getRotation().getDegrees()+45);
        }
        else{
            return Rotation2d.fromDegrees(inputs.colorVisionData.averageYaw()+ drive.getRotation().getDegrees());
        }
    }
}
