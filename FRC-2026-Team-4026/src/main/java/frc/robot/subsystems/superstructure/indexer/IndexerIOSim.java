package frc.robot.subsystems.superstructure.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class IndexerIOSim implements IndexerIO {
    public final DCMotorSim leftGearbox;
    public final DCMotorSim rightGearbox;
    double leftRadians = 0.0;
    double rightRadians = 0.0;
    private boolean closedLoop = false;
    public IndexerIOSim() {
       rightGearbox =  new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001,0.001),DCMotor.getKrakenX44(1).withReduction(0));
       leftGearbox =  new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001,0.001),DCMotor.getKrakenX44(1).withReduction(0));
       SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        //right and left gearbox will be the same voltage approximately
        Voltage simVoltage = Volts.of(rightGearbox.getInputVoltage()); 
        simVoltage = SimulatedBattery.clamp(simVoltage);
        inputs.indexerData = new IndexerIOData(true, true, rightGearbox.getInputVoltage(), leftGearbox.getInputVoltage(), leftGearbox.getCurrentDrawAmps(), rightGearbox.getCurrentDrawAmps(),0.0,0.0,0.0,0.0);
    }

    public void runOpenLoop(double voltage){
        closedLoop = false;
    }

    public Current getSupplyCurrent(){
        // same as previous comment, both sides should be approximately equal
        return Amps.of(rightGearbox.getCurrentDrawAmps());
    }


}
