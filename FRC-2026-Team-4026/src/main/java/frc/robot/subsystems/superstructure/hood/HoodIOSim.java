package frc.robot.subsystems.superstructure.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
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
        Voltage realVoltage =  Volts.of(hoodSim.getInputVoltage());
        realVoltage = SimulatedBattery.clamp(realVoltage);
        inputs.hoodData = new HoodIOData(
            true,
            realVoltage.in(Volts),
            realPosition.in(Rotations),
            hoodSim.getCurrentDrawAmps(),
            hoodSim.getCurrentDrawAmps()
            );
    }

    public Current getSupplyCurrent(){
        return Amps.of(hoodSim.getCurrentDrawAmps());
    }

    public void setVoltage(double voltage){
        this.voltage = Volts.of(voltage);
    }

    public void setPosition(double position){
        hoodSim.setAngle(position*gearingRatio);

    }
}