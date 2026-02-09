package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOData;
import frc.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOInputs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ShooterIOSim implements ShooterIO {
    private final LinearSystem<N1, N1, N1> flywheelSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.05, 10);

    private FlywheelSim shooterSim; 
    private final SimulatedMotorController.GenericMotorController motorLeft, motorRight;

    private final PIDController controller = new PIDController (0,0,0);

    private Voltage targetVoltage = Volts.zero();

    public ShooterIOSim(
    ) {
        motorLeft = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1));
        motorRight = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1));
        shooterSim = new FlywheelSim(flywheelSystem, DCMotor.getKrakenX60(2),0.0);

        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        shooterSim.update(0.0);

       }


    public Current getSupplyCurrent () {
        return Amps.of(shooterSim.getCurrentDrawAmps());
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        AngularVelocity realVelocity = RotationsPerSecond.of(shooterSim.getAngularVelocityRPM()/10);
        Voltage leftVoltage = SimulatedBattery.clamp(targetVoltage);
        inputs.data = new ShooterIOData (true,true,shooterSim.getAngularVelocityRPM(), shooterSim.getAngularVelocityRPM(), shooterSim.getInputVoltage(), shooterSim.getInputVoltage(), getSupplyCurrent().in(Amps), getSupplyCurrent().in(Amps));
    
}

    @Override
    public void setVoltage (double voltage) {
        targetVoltage = Volts.of(voltage);
        shooterSim.setInputVoltage(targetVoltage.in(Volts));
    }

    @Override
    public void setVelocity(double velocity){
        shooterSim.setAngularVelocity(velocity);
    }
      

}
