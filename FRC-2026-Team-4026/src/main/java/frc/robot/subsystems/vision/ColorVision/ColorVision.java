package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;

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
        Logger.recordOutput("averageYaw",getAverageYaw());

    }
    public Rotation2d getAverageYaw(){

        return Rotation2d.fromDegrees(inputs.colorVisionData.averageYaw());
    }
}
