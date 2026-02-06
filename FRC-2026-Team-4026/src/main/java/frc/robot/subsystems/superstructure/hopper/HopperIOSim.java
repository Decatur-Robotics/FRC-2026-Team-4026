package frc.robot.subsystems.superstructure.hopper;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOData;
import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOInputs;
import static edu.wpi.first.units.Units.Volts;


public class HopperIOSim implements HopperIO {


    private final LinearSystem<N1, N1, N1> flywheelSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0, 10);

    private FlywheelSim hopperSim; 
    private final SimulatedMotorController.GenericMotorController motorLeft, motorRight;

    private final PIDController controller = new PIDController (0,0,0);

    private Voltage targetVoltage = Volts.zero();

    public HopperIOSim(
    ) {
        motorLeft = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1));
        motorRight = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1));
        hopperSim = new FlywheelSim(flywheelSystem, DCMotor.getKrakenX60(2),0.0);
        hopperSim.update(0.0);

       }

    public void updateInputs(HopperIOInputs inputs) {
        Voltage leftVoltage = SimulatedBattery.clamp(targetVoltage);

        
    
        
        inputs.data = new HopperIOData (true,true,0.0,0.0,0.0,0.0);
    
}

    public void setVoltage (double voltage) {
        targetVoltage = Volts.of(voltage);
    }
}