package frc.robot.subsystems.superstructure.hopper;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class Hopper extends SubsystemBase{

    private double voltage;
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

public Hopper (HopperIO io) {
    this.io = io;
    voltage = inputs.data.leftVoltage();
}
public double getCurrent() {
    return inputs.data.supplyCurrentLeft();
}

public Command setVoltageCommand(double voltage) {
    return Commands.run(() -> io.setVoltage(voltage));
}
public void periodic () {
    io.updateInputs(inputs);
}

}
