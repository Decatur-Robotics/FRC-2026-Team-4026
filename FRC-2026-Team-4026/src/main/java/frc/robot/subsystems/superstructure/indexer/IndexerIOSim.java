package frc.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.*;

public class IndexerIOSim implements IndexerIO {
    //i know that something's probably missing here, so comment what it is if you know.
    public final DCMotor leftGearbox;
    public final DCMotor rightGearbox;
    double leftTorque = 0.0;
    double rightTorque = 0.0;
    double leftRadians = 0.0;
    double rightRadians = 0.0;
    private boolean closedLoop = false;
    public IndexerIOSim() {
       rightGearbox = DCMotor.getKrakenX44(1).withReduction(0);
       leftGearbox = DCMotor.getKrakenX44(1).withReduction(0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
       inputs.indexerData = new IndexerIOData(true, true, rightGearbox.getVoltage(rightTorque, rightRadians), leftGearbox.getVoltage(leftTorque, leftRadians), leftGearbox.getCurrent(leftTorque), rightGearbox.getCurrent(rightTorque));
    }

    public void runOpenLoop(double voltage){
        closedLoop = false;
    }
}
