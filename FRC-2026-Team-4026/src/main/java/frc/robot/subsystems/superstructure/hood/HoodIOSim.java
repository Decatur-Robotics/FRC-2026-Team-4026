package frc.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.subsystems.superstructure.hood.HoodIO.HoodIOInputs;
public class HoodIOSim implements HoodIO {

    private final MapleMotorSim hoodSim;
    private final SimulatedMotorController hoodMotorController;

    public HoodIOSim(){
        this.hoodSim = new MapleMotorSim(new SimMotorConfigs(DCMotor.getMinion(1), 30, MomentOfInertia.ofBaseUnits(0, KilogramSquareMeters),Voltage.ofBaseUnits(0, Volts) ));
        hoodMotorController = new SimulatedMotorController.GenericMotorController(DCMotor.getMinion(1));
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        hoodSim.update(Time.ofBaseUnits(0, Seconds));
        hoodSim.useMotorController(hoodMotorController);
    }



    @Override
    public void updateInputs(HoodIOInputs inputs){      
        Voltage realVoltage = hoodSim.getAppliedVoltage();
        realVoltage = SimulatedBattery.clamp(realVoltage);
        inputs.hoodData = new HoodIOData(
            true,
            realVoltage.in(Volts),
            hoodSim.getAngularPosition().in(Rotations),
            hoodSim.getSupplyCurrent().in(Amps),
            hoodSim.getStatorCurrent().in(Amps)
            );
    }

    public Current getSupplyCurrent(){
        return Amps.of(hoodSim.getStatorCurrent().in(Amps));
    }

    public void setPosition(double position){

    }
}