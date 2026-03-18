package frc.robot.subsystems.superstructure.shooter;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {


    private int ballsShot;
    private boolean shootingBall;
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(1.5),Seconds.of(10), (state) -> SignalLogger.writeString("state", state.toString())),
    new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));


public Shooter (ShooterIO io) {
    this.io = io;
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

public Command shootAimCommand(){
    return Commands.run(() -> io.setVelocity(ShotEstimator.getInstance().getTargetVelocity().get()));
}
public Command shootOnMoveCommand(Drive drive){
    return Commands.run(()-> io.setVelocity(ShotEstimator.getInstance().getTargetVelocity(drive.getDistanceToHub(drive.getFuturePose().get()).getAsDouble()).get()));
}

public Command setVoltageCommand(double voltage) {
    return Commands.run(() -> io.setVoltage(voltage));
}

@Override
public void periodic () {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if(getCurrent() > 40){
        if(shootingBall == false){
            ballsShot+=1;
        }
        shootingBall = true;
    } else if(getCurrent() < 10){
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