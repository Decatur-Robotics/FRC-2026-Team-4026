
package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    public TalonFX motor;
    private TalonFXConfiguration config;

    private StatusSignal<Voltage> voltage;

    private VoltageOut voltageRequest;

    private VelocityVoltage velocityRequest;

    private double velocity;



    public ShooterIOTalonFX () {
        motor = new TalonFX(Ports.SHOOTER_MOTOR);
        
        config = new TalonFXConfiguration().withSlot0(ShooterConstants.SLOT_0_CONFIGS);
        motor.getConfigurator().apply(config);
        voltage = motor.getMotorVoltage();
        velocity = motor.getVelocity().getValueAsDouble();
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
        velocityRequest = new VelocityVoltage(velocity);

        tryUntilOk(5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, voltage, motor.getVelocity()));
        tryUntilOk(5, () -> motor.optimizeBusUtilization(40));
        PhoenixUtil.registerSignals(true,voltage);    
    }

@Override
public void setVelocity (double velocity){
    this.velocity = velocity;
     motor.setControl(velocityRequest.withVelocity(velocity)); 
    Logger.recordOutput("Target Velocity", velocity);
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
        motor.getVelocity().getValueAsDouble(),
        motor.getMotorVoltage().getValueAsDouble(),
        motor.getSupplyCurrent().getValueAsDouble(),
        motor.getDeviceTemp().getValueAsDouble()
        );
}

}
