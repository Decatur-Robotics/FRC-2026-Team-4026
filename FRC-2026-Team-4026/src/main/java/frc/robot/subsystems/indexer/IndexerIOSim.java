package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.*;

public class IndexerIOSim implements IndexerIO {
    //i know that something's probably missing here, so comment what it is if you know.
    public final DCMotor gearbox;
    private boolean closedLoop = false;
    public IndexerIOSim() {
       gearbox = DCMotor.getKrakenX44(2).withReduction(0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
       inputs.indexerData = new IndexerIOData(true, true, 0.0, 0.0, 0.0, 0.0);
    }

    public void runOpenLoop(double voltage){
        closedLoop = false;
    }
}
