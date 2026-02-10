
package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    public TalonFX motor;
    private TalonFXConfiguration config;

    private StatusSignal<Voltage> voltage;

    private VoltageOut voltageRequest;

    private MotionMagicVelocityDutyCycle velocityRequest;

    private StatusSignal<AngularVelocity> velocity;



    public ShooterIOTalonFX () {
        motor = new TalonFX(Ports.SHOOTER_MOTOR);
        
        config = new TalonFXConfiguration ();

        voltage = motor.getMotorVoltage();
    
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
        
        velocityRequest = new MotionMagicVelocityDutyCycle(velocity.getValueAsDouble());

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(config);
        velocity = motor.getVelocity();

        tryUntilOk(5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, voltage, velocity));
        tryUntilOk(5, () -> motor.optimizeBusUtilization());
        PhoenixUtil.registerSignals(true, velocity,voltage);    
    }

@Override
public void setVelocity (double velocity){
    motor.setControl(velocityRequest.withVelocity(velocity)); 
}

@Override
public void setVoltage (double voltage){
    motor.setVoltage(voltage);
}

@Override
public void periodic () {
    if (motor.hasResetOccurred()) {
        motor.optimizeBusUtilization();
        motor.getVelocity().setUpdateFrequency(40);
    }
}

@Override
public void updateInputs (ShooterIOInputs inputs){
    inputs.data = new ShooterIOData (
        motor.isConnected(),
        motor.getSupplyVoltage().getValueAsDouble(),
        motor.getVelocity().getValueAsDouble(),
        motor.getSupplyCurrent().getValueAsDouble(),
        motor.getDeviceTemp().getValueAsDouble()
        );
}

}
