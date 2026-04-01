package frc.robot.subsystems.superstructure.shooter;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Elastic;

public class Shooter extends SubsystemBase {
  private ShuffleboardTab shooterTab;
  private GenericEntry slider;



    private int ballsShot;
    private boolean shootingBall;
    private ShooterIO io;
    private double sliderVelocity;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(1.5),Seconds.of(10), (state) -> SignalLogger.writeString("state", state.toString())),
    new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));


public Shooter (ShooterIO io) {
      shooterTab=Shuffleboard.getTab("shooter");
    slider = shooterTab
   .add("Speed", 1)
   .withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 100)) // specify widget properties here
   .getEntry();

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
    System.out.println(velocity);
    return Commands.runOnce(() -> io.setVelocity(velocity));
}

public Command shootAimCommand(){
    return Commands.run(() -> io.setVelocity(ShotEstimator.getInstance().getTargetVelocity().get()));
}
public Command shootOnMoveCommand(Drive drive){
    return Commands.run(()-> io.setVelocity(ShotEstimator.getInstance().getTargetVelocity(drive.getDistanceToHub(drive.convergentFlightTime().get()).getAsDouble()).get()));
}

public Command passAimCommand(){
    return Commands.run(()->io.setVelocity(ShotEstimator.getInstance().getPassingVelocity().get()));
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
    sliderVelocity = slider.getDouble(0);



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

public Command setVelocityWithSlider(){
    return Commands.runOnce(() ->io.setVelocity(sliderVelocity));
}
public double getSlider(){
    return sliderVelocity;
}
}