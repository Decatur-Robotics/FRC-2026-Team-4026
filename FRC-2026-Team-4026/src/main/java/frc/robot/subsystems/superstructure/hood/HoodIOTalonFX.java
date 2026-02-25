package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Ports;
public class HoodIOTalonFX implements HoodIO{
    public TalonFX motor;
    public TalonFXConfiguration config ;

    private MotionMagicVoltage positionRequest;
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

        position = motor.getPosition().getValueAsDouble();
        voltage = motor.getMotorVoltage();
        supplyAmps = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();

        positionRequest = new MotionMagicVoltage(position);
        voltageRequest = new VoltageOut(voltage.getValueAsDouble());
    }
    
    @Override
    public void periodic(){
        if(motor.hasResetOccurred()){
            motor.optimizeBusUtilization();
            motor.getPosition().setUpdateFrequency(40);
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs){
        inputs.hoodData = new HoodIOData(
            motor.isConnected(),
            voltage.getValueAsDouble(),
            position,
            supplyAmps.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble()
            );
    }

    @Override
    public void setPosition(double position){
        this.position = position;
        motor.setPosition(position);
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
