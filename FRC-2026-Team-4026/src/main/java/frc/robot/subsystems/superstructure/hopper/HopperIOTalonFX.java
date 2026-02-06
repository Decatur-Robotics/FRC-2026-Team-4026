package frc.robot.subsystems.superstructure.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOData;
import frc.robot.subsystems.superstructure.hopper.HopperIO.HopperIOInputs;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO{
    public TalonFX motorLeft, motorRight;
    private TalonFXConfiguration config;

    private StatusSignal<Voltage> voltage;

    private VoltageOut voltageRequest;

    public HopperIOTalonFX () {
        motorLeft = new TalonFX(Ports.HOPPER_MOTOR_LEFT);
        motorRight = new TalonFX (Ports.HOPPER_MOTOR_RIGHT);
        
        config = new TalonFXConfiguration ();

        voltage = motorLeft.getMotorVoltage();
    
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
            
        motorLeft.getConfigurator().apply(config);
        motorRight.getConfigurator().apply(config);
        motorRight.setControl(new Follower(Ports.HOPPER_MOTOR_LEFT, MotorAlignmentValue.Opposed));

        tryUntilOk(
            5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, voltage)
        );

         tryUntilOk(
             5, () -> motorLeft.optimizeBusUtilization());

        PhoenixUtil.registerSignals(true,voltage);    
    }

@Override
public void setVoltage (double voltage){
    motorLeft.setVoltage(voltage);
    motorRight.setVoltage(voltage);
}

@Override
public void periodic () {
    if (motorLeft.hasResetOccurred()||     motorRight.hasResetOccurred()) {
        motorLeft.optimizeBusUtilization(); motorRight.optimizeBusUtilization();
    }
}

@Override
public void updateInputs (HopperIOInputs inputs){
    inputs.data = new HopperIOData (
        motorLeft.isConnected(),
        motorRight.isConnected(),
        motorLeft.getSupplyVoltage().getValueAsDouble(),
        motorRight.getSupplyVoltage().getValueAsDouble(),
        motorLeft.getDeviceTemp().getValueAsDouble(),
        motorRight.getDeviceTemp().getValueAsDouble()
        );
}
}
