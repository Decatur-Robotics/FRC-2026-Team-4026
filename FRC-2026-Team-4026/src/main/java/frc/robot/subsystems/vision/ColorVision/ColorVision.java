package frc.robot.subsystems.vision.ColorVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.subsystems.drive.Drive;

public class ColorVision {
    private ColorVisionIO io;
    private final ColorVisionIOInputsAutoLogged inputs = new ColorVisionIOInputsAutoLogged();
    private double yaw;
    private final ColorVisionConsumer colorVisionConsumer;

    public ColorVision(ColorVisionConsumer consumer, ColorVisionIO io){
        this.io = io;
        yaw = inputs.colorVisionData.yaw();
        colorVisionConsumer = consumer;

    }
    public void periodic(){
        io.updateInputs(inputs);
        colorVisionConsumer.accept(inputs.colorVisionData.yaw());
        Logger.processInputs("ColorVision", inputs);
        Logger.recordOutput("yaw",inputs.colorVisionData.yaw());
        

    }

    
    @FunctionalInterface
    public interface ColorVisionConsumer{
        void accept(double yaw);
    }

}
