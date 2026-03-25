package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO{
    public TalonFX motor;
    public TalonFXConfiguration config ;

    private PositionDutyCycle positionRequest;
    private VoltageOut voltageRequest;

    private double position;
    private StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyAmps;
    private final StatusSignal<Current> torqueCurrent;

    private CANcoder encoder;
    
    public HoodIOTalonFX(){
        motor = new TalonFX(Ports.HOOD_MOTOR);
        encoder = new CANcoder(Ports.HOOD_ENCODER);
        config = new TalonFXConfiguration().withSlot0(HoodConstants.SLOT0_CONFIGS);
        motor.getConfigurator().apply(config);
        encoder.setPosition(0);
        position = motor.getPosition().getValueAsDouble();
        voltage = motor.getMotorVoltage();
        supplyAmps = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();

        positionRequest = new PositionDutyCycle(position);
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
        PhoenixUtil.registerSignals(false, voltage, supplyAmps, torqueCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(40, motor.getPosition(), voltage, supplyAmps, torqueCurrent, motor.getVelocity());
        motor.optimizeBusUtilization();
    }
    
    @Override
    public void periodic(){
        if(motor.hasResetOccurred()){
            motor.optimizeBusUtilization();
            motor.getPosition().setUpdateFrequency(40);
        }
        //motor.setPosition(encoder.getPosition().getValueAsDouble());
    }

    @Override
    public void updateInputs(HoodIOInputs inputs){
        inputs.hoodData = new HoodIOData(
            motor.isConnected(),
            motor.getMotorVoltage().getValueAsDouble(),
            motor.getPosition().getValueAsDouble(),
            motor.getVelocity().getValueAsDouble(),
            encoder.getPosition().getValueAsDouble(),
            supplyAmps.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble(),
            motor.getAcceleration().getValueAsDouble()
            );
    }

    @Override
    public void setPosition(double position){
        this.position = position;
        motor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void setVoltage(double voltage){
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop(){
        motor.stopMotor();
    }

}
