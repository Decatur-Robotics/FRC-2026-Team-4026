package frc.robot.subsystems.superstructure.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;

public class IndexerIOTalonFX implements IndexerIO{
    private TalonFX mecanumMotor, beltMotor, kickMotor;
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageRequest;

    private StatusSignal<Voltage> mecanumVoltage;
    private StatusSignal<Voltage> beltVoltage;
    private StatusSignal<Voltage> kickVoltage;

    private StatusSignal<Current> mechanumCurrent;
    private StatusSignal<Current> beltCurrent;
    private StatusSignal<Current> kickCurrent;

    public IndexerIOTalonFX(){
        mecanumMotor = new TalonFX(Ports.INDEXER_MOTOR_MECANUM);
        beltMotor = new TalonFX(Ports.INDEXER_MOTOR_BELT); 
        kickMotor = new TalonFX(Ports.INDEXER_MOTOR_KICK);

        //idk if alligned or opposed
        beltMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECANUM, MotorAlignmentValue.Aligned));
        kickMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECANUM, MotorAlignmentValue.Aligned));

        mecanumVoltage = mecanumMotor.getMotorVoltage();
        beltVoltage = beltMotor.getMotorVoltage();
        kickVoltage = kickMotor.getMotorVoltage();

        mechanumCurrent = mecanumMotor.getSupplyCurrent();
        beltCurrent = beltMotor.getSupplyCurrent();
        kickCurrent = kickMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(40, mecanumVoltage, beltVoltage,kickVoltage);
    }

    @Override
    public void periodic(){
        if(mecanumMotor.hasResetOccurred() || beltMotor.hasResetOccurred() || kickMotor.hasResetOccurred()){
            mecanumMotor.optimizeBusUtilization(40);
            beltMotor.optimizeBusUtilization(40);
            kickMotor.optimizeBusUtilization(40);
            mecanumMotor.getPosition().setUpdateFrequency(40);
        }
        
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        inputs.indexerData = new IndexerIO.IndexerIOData(
            mecanumMotor.isConnected(),
            beltMotor.isConnected(),
            kickMotor.isConnected(),
            mechanumCurrent.getValueAsDouble(),
            beltCurrent.getValueAsDouble(),
            kickCurrent.getValueAsDouble(),
            mecanumVoltage.getValueAsDouble(),
            beltVoltage.getValueAsDouble(),
            kickVoltage.getValueAsDouble(),
            mecanumMotor.getVelocity().getValueAsDouble(),
            beltMotor.getVelocity().getValueAsDouble(),
            kickMotor.getVelocity().getValueAsDouble(),
            mecanumMotor.getDeviceTemp().getValueAsDouble(),
            beltMotor.getDeviceTemp().getValueAsDouble(),
            kickMotor.getDeviceTemp().getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        voltageRequest = new VoltageOut(voltage);
        mecanumMotor.setControl(voltageRequest);
        beltMotor.setControl(voltageRequest);
        kickMotor.setControl(voltageRequest);
    }

    @Override
    public void stop(){
        mecanumMotor.stopMotor(); 
        beltMotor.stopMotor();
        kickMotor.stopMotor();
    }


}
