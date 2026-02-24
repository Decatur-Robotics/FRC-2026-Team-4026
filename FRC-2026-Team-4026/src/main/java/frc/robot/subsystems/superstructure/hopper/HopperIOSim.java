// package frc.robot.subsystems.superstructure.hopper;

// import org.ironmaple.simulation.motorsims.SimulatedBattery;
// import org.ironmaple.simulation.motorsims.SimulatedMotorController;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOData;
// import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOInputs;
// import static edu.wpi.first.units.Units.*;


// public class HopperIOSim implements HopperIO {


//     private final LinearSystem<N1, N1, N1> flywheelSystem = LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(2), 0, 10);

//     private FlywheelSim hopperSim; 
//     private final SimulatedMotorController.GenericMotorController motorLeft, motorRight;

//     private final PIDController controller = new PIDController (0,0,0);

//     private Voltage targetVoltage = Volts.zero();

//     public HopperIOSim(
//     ) {
//         motorLeft = new SimulatedMotorController.GenericMotorController(DCMotor.getNeo550(1));
//         motorRight = new SimulatedMotorController.GenericMotorController(DCMotor.getNeo550(1));
//         hopperSim = new FlywheelSim(flywheelSystem, DCMotor.getNeo550(2),0.0);
//         hopperSim.update(0.0);
//         SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);

//        }

//     public void updateInputs(HopperIOInputs inputs) {
//         Voltage leftVoltage = SimulatedBattery.clamp(targetVoltage);

//         inputs.data = new HopperIOData (true,true, leftVoltage.in(Volts),leftVoltage.in(Volts),getSupplyCurrent().in(Amps), getSupplyCurrent().in(Amps),0.0,0.0);


//     }
//     public Current getSupplyCurrent () {
//         return Amps.of(hopperSim.getCurrentDrawAmps());
//     }

//     public void setVoltage (double voltage) {
//         targetVoltage = Volts.of(voltage);
//     }
// }