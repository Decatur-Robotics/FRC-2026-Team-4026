package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.Logger;

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
    public float getAverageYaw(){

        return inputs.colorVisionData.averageYaw();
    }
}
