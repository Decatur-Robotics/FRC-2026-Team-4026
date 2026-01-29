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
import frc.robot.Ports;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    public TalonFX motorLeft, motorRight;
    private TalonFXConfiguration config;

    private StatusSignal<Voltage> voltage;

    private VoltageOut voltageRequest;

    private MotionMagicVelocityDutyCycle velocityRequest;

    private StatusSignal<AngularVelocity> velocity;



    public ShooterIOTalonFX () {
        motorLeft = new TalonFX(Ports.SHOOTER_MOTOR_LEFT);
        motorRight = new TalonFX (Ports.SHOOTER_MOTOR_RIGHT);
        
        config = new TalonFXConfiguration ();

        voltage = motorLeft.getMotorVoltage();
    
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
        
        velocityRequest = new MotionMagicVelocityDutyCycle(velocity.getValueAsDouble());

        motorLeft.getConfigurator().apply(config);
        motorRight.getConfigurator().apply(config);
        motorRight.setControl(new Follower(Ports.SHOOTER_MOTOR_LEFT, MotorAlignmentValue.Opposed));

         velocity = motorLeft.getVelocity();

        tryUntilOk(
            5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, voltage, velocity)
        );

         tryUntilOk(
             5, () -> motorLeft.optimizeBusUtilization());

        PhoenixUtil.registerSignals(true, velocity,voltage);    
    }

@Override
public void setVelocity (double velocity){
    motorLeft.setControl(velocityRequest.withVelocity(velocity)); 
}

@Override
public void setVoltage (double voltage){
    motorLeft.setVoltage(voltage);
}

@Override
public void periodic () {
    if (motorLeft.hasResetOccurred()||     motorRight.hasResetOccurred()) {
        motorLeft.optimizeBusUtilization(); motorRight.optimizeBusUtilization(); motorLeft.getVelocity().setUpdateFrequency(40);
    }
}

@Override
public void updateInputs (ShooterIOInputs inputs){
    inputs.data = new ShooterIOData (motorLeft.isConnected(),motorRight.isConnected(), motorLeft.getSupplyVoltage().getValueAsDouble(), motorRight.getSupplyVoltage().getValueAsDouble(), motorLeft.getVelocity().getValueAsDouble(), motorRight.getVelocity().getValueAsDouble(), motorLeft.getSupplyCurrent().getValueAsDouble(), motorRight.getSupplyCurrent().getValueAsDouble());
}

}
