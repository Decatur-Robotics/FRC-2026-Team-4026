package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO{
    private final DCMotorSim climberSim;
    private final double gearingRatio = 200;
    public ClimberIOSim(){
        climberSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.001, 0.001),DCMotor.getKrakenX60(2));
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        climberSim.update(0);

    }


    public void updateInputs(ClimberIOInputs inputs){
        inputs.climberData = new ClimberIOData(
            true,
            true,
            climberSim.getAngularPosition().magnitude()*gearingRatio,
            climberSim.getInputVoltage(),
            getSupplyCurrent().in(Amps),
            0);
    }   

    public Current getSupplyCurrent(){
        return Amps.of(climberSim.getCurrentDrawAmps());
    }

    public void setvoltage(double voltage){
        climberSim.setInputVoltage(voltage);
    }

    public void setPosition(double position){
        climberSim.setAngle(position/gearingRatio);
    }
}
