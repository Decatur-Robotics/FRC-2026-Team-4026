package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class ShooterIOSim implements ShooterIO {
    private final LinearSystem<N1, N1, N1> flywheelSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.05, 10);

    private FlywheelSim shooterSim; 
    private double velocity;

    private Voltage targetVoltage = Volts.zero();

    public ShooterIOSim(
    ) {

        shooterSim = new FlywheelSim(flywheelSystem, DCMotor.getKrakenX60(2),0.0);

        velocity = 0;
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        shooterSim.update(0.0);

       }


    public Current getSupplyCurrent () {
        return Amps.of(shooterSim.getCurrentDrawAmps());
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        Voltage voltage = SimulatedBattery.clamp(targetVoltage);

        inputs.data = new ShooterIOData (
            true,
            velocity,
            voltage.in(Volts),
            getSupplyCurrent().in(Amps),
            0.0
            );
    
}

    @Override
    public void setVoltage (double voltage) {
        targetVoltage = Volts.of(voltage);
        shooterSim.setInputVoltage(targetVoltage.in(Volts));
    }

    @Override
    public void setVelocity(double velocity){
        this.velocity = velocity;
    }
      

}
