package frc.robot.subsystems.vision.ColorVision.HopperVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ColorVision.ColorVisionIO;

public class HopperVision {
    private HopperVisionIO io;
private final HopperVisionIOInputsAutoLogged inputs = new HopperVisionIOInputsAutoLogged();
    private double ballsInHopper;
    private double percentAreaOfHopperFilled;

    public HopperVision(HopperVisionIO io){
        this.io = io;
        ballsInHopper = inputs.hopperVisionData.ballsInHopper();
        percentAreaOfHopperFilled = inputs.hopperVisionData.percentAreaOfHopperFilled();

    }
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("HopperVision", inputs);
        Logger.recordOutput("Balls in Hopper",inputs.hopperVisionData.ballsInHopper());
        Logger.recordOutput("Percent Area of Hopper Filled", inputs.hopperVisionData.percentAreaOfHopperFilled());

    }
    public double getBallsInHopper(){
        return inputs.hopperVisionData.ballsInHopper();
    }

}
