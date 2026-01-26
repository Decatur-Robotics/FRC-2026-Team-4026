package frc.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Ports;
import frc.robot.shooter.ShooterIO.ShooterIOData;
import frc.robot.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX {
    public TalonFX motorLeft, motorRight;
    private TalonFXConfiguration config;

    private StatusSignal<Voltage> voltage;

    private VoltageOut voltageRequest;

    private MotionMagicVelocityDutyCycle velocityRequest;

    private StatusSignal<Velocity> velocity;



    public ShooterIOTalonFX () {
        motorLeft = new TalonFX( Ports.SHOOTER_MOTOR_LEFT);
        motorRight = new TalonFX ( Ports.SHOOTER_MOTOR_RIGHT);
        
        config = new TalonFXConfiguration ();

        voltage = motorLeft.getMotorVoltage();
    
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
        
        velocityRequest = new MotionMagicVelocityDutyCycle(velocity.getValueAsDouble());

        motorLeft.getConfigurator().apply(config);
        motorRight.getConfigurator().apply(config);

         velocity = motorLeft.getVelocity();

        tryUntilOk(
            5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, voltage, velocity)
        );

         tryUntilOk(
             5, () -> motorLeft.optimizeBusUtilization());

        PhoenixUtil.registerSignals(true, velocity,voltage);    
    }
public void setVelocity (
    double velocity
)
{motorLeft.setControl(velocityRequest.withVelocity(velocity));
motorRight.setControl(velocityRequest.withVelocity(velocity)); 
}


public void setVoltage (
    double voltage
)
{motorLeft.setVoltage(voltage);
motorRight.setVoltage(voltage);
}


public void periodic () {
    if (motorLeft.hasResetOccurred()||     motorRight.hasResetOccurred()) {
        motorLeft.optimizeBusUtilization(); motorRight.optimizeBusUtilization(); motorLeft.getVelocity().setUpdateFrequency(40);
    }
}

public void updateInputs (
    ShooterIOInputs inputs
)
{
    inputs.data = new ShooterIOData (motorLeft.isConnected(),motorRight.isConnected(), motorLeft.getSupplyVoltage().getValueAsDouble(), motorRight.getSupplyVoltage().getValueAsDouble(), motorLeft.getVelocity().getValueAsDouble(), motorRight.getVelocity().getValueAsDouble());
}

}
