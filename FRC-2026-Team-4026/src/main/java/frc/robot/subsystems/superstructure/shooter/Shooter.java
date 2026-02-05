package frc.robot.subsystems.superstructure.shooter;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private double velocity;
    private double voltage;

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

public Shooter (ShooterIO io) {
    this.io = io;
    velocity = ShooterConstants.FUEL_REST_VELOCITY;
    voltage = inputs.data.leftVoltage();
}

public double getVelocity() {
    return inputs.data.leftVelocity();
}

public double getCurrent() {
    return inputs.data.supplyCurrentLeft();
}

public Command setVelocityCommand(double velocity) {
    return Commands.run(() -> io.setVelocity(velocity));
}

public Command setVoltageCommand(double voltage) {
    return Commands.run(() -> io.setVoltage(voltage));
}

@Override
public void periodic () {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
}
}
