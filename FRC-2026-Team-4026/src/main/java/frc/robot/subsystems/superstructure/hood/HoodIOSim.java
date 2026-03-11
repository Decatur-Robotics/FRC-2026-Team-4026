package frc.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {

    private final DCMotorSim hoodSim;
    private final double gearingRatio = 30;
    private Voltage voltage;

    public HoodIOSim(){
        this.hoodSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001, 0.001),DCMotor.getMinion(1));
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        hoodSim.update(0);

        voltage = Volts.of(hoodSim.getInputVoltage());
    }



    @Override
    public void updateInputs(HoodIOInputs inputs){      
    
        Angle realPosition = Rotations.of(hoodSim.getAngularPosition().magnitude()/gearingRatio);
        // Voltage realVoltage =  Volts.of(hoodSim.getInputVoltage());
        // realVoltage = SimulatedBattery.clamp(realVoltage);
        inputs.hoodData = new HoodIOData(
            true,
            hoodSim.getInputVoltage(),
            realPosition.in(Rotations),
            0.0,
            0.0,
            hoodSim.getCurrentDrawAmps(),
            hoodSim.getCurrentDrawAmps(),
            0.0,
            0.0
            );
    }

    public Current getSupplyCurrent(){
        return Amps.of(hoodSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(double voltage){
        hoodSim.setInputVoltage(voltage);
    }

    @Override
    public void setPosition(double position){
        hoodSim.setAngle(position*gearingRatio);

    }
}