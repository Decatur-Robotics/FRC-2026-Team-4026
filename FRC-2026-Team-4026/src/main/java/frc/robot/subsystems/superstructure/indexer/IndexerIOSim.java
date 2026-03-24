package frc.robot.subsystems.superstructure.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class IndexerIOSim implements IndexerIO {
    public final DCMotorSim mechanumMotorSim;
    public final DCMotorSim beltMotorSim;
    public final DCMotorSim kickMotorSim;
    double leftRadians = 0.0;
    double rightRadians = 0.0;

    public IndexerIOSim() {
       mechanumMotorSim =  new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001,0.001),DCMotor.getKrakenX44(1).withReduction(0));
       beltMotorSim =  new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001,0.001),DCMotor.getKrakenX44(1).withReduction(0));
       kickMotorSim =  new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001,0.001),DCMotor.getKrakenX44(1).withReduction(0));
       SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        //right and left gearbox will be the same voltage approximately
        Voltage simVoltage = Volts.of(mechanumMotorSim.getInputVoltage()); 
        simVoltage = SimulatedBattery.clamp(simVoltage);
        inputs.indexerData = new IndexerIOData(
            true,
            true,
            true,
            mechanumMotorSim.getInputVoltage(),
            beltMotorSim.getInputVoltage(),
            kickMotorSim.getInputVoltage(),
            mechanumMotorSim.getCurrentDrawAmps(),
            beltMotorSim.getCurrentDrawAmps(),
            kickMotorSim.getCurrentDrawAmps(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
            );
    }


    public Current getSupplyCurrent(){
        // same as previous comment, both sides should be approximately equal
        return Amps.of(mechanumMotorSim.getCurrentDrawAmps());
    }

    public void setVoltage(double voltage){
        mechanumMotorSim.setInputVoltage(voltage);
        beltMotorSim.setInputVoltage(voltage);
        kickMotorSim.setInputVoltage(voltage);
    }

}
