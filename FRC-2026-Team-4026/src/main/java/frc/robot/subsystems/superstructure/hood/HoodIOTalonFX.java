package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
public class HoodIOTalonFX implements HoodIO{
    public TalonFX motor;
    public TalonFXConfiguration config ;

    private MotionMagicVoltage positionRequest;
    private VoltageOut voltageRequest;

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyAmps;
    private final StatusSignal<Current> torqueCurrent;
    
    public HoodIOTalonFX(){
        motor = new TalonFX(Ports.HOOD_MOTOR);

        config = new TalonFXConfiguration();
        config.Slot0 = new Slot0Configs()
        .withKP(HoodConstants.kP)
        .withKI(HoodConstants.kI)
        .withKD(HoodConstants.kD)
        .withKS(HoodConstants.kS)
        .withKV(HoodConstants.kV)
        .withKA(HoodConstants.kA)
        .withKP(HoodConstants.kP);

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        supplyAmps = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();

        positionRequest = new MotionMagicVoltage(position.getValueAsDouble());
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
            position.getValueAsDouble(),
            voltage.getValueAsDouble(),
            supplyAmps.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble()
            );
    }

    @Override
    public void setPosition(double position){
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

    // This cant do anything, i'll fix later
    @Override
    public void setPID(HoodConstants constants){
        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kS = constants.kS;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kG = constants.kG;
    }
}
