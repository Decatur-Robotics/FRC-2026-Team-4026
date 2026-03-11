package frc.robot.subsystems.superstructure.shooter;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {

    private double velocity;
    private double voltage;

    private int ballsShot;
    private boolean shootingBall;
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(1.5),Seconds.of(10), (state) -> SignalLogger.writeString("state", state.toString())),
    new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));


public Shooter (ShooterIO io) {
    this.io = io;
    velocity = ShooterConstants.FUEL_REST_VELOCITY;
    voltage = inputs.data.voltage();
    ballsShot = 0;
    shootingBall = false;
}

public double getVelocity() {
    return inputs.data.velocity();
}

public double getCurrent() {
    return inputs.data.supplyCurrent();
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
    if(getCurrent() > 10){
        if(shootingBall == false){
            ballsShot++;
        }
        shootingBall = true;
    } else if(getCurrent() < 5){
        shootingBall = false;
    }

    Logger.recordOutput("Shooter/Balls Shot", ballsShot);
}

public Command sysIdQuasistatic (SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic (SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

public int getNumBallsShot(){
    return ballsShot;
}
}