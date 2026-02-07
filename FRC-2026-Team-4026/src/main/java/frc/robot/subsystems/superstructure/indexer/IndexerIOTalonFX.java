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
    private TalonFX mechanumMotor, beltMotor, kickMotor;
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageRequest;

    private StatusSignal<Voltage> mechanumVoltage;
    private StatusSignal<Voltage> beltVoltage;
    private StatusSignal<Voltage> kickVoltage;

    private StatusSignal<Current> mechanumCurrent;
    private StatusSignal<Current> beltCurrent;
    private StatusSignal<Current> kickCurrent;

    public IndexerIOTalonFX(){
        mechanumMotor = new TalonFX(Ports.INDEXER_MOTOR_MECHANUM);
        beltMotor = new TalonFX(Ports.INDEXER_MOTOR_BELT); 
        kickMotor = new TalonFX(Ports.INDEXER_MOTOR_KICK);

        //idk if alligned or opposed
        beltMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECHANUM, MotorAlignmentValue.Aligned));
        kickMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECHANUM, MotorAlignmentValue.Aligned));

        mechanumVoltage = mechanumMotor.getMotorVoltage();
        beltVoltage = beltMotor.getMotorVoltage();
        kickVoltage = kickMotor.getMotorVoltage();

        mechanumCurrent = mechanumMotor.getSupplyCurrent();
        beltCurrent = beltMotor.getSupplyCurrent();
        kickCurrent = kickMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(40, mechanumVoltage, beltVoltage,kickVoltage);
    }

    @Override
    public void periodic(){
        if(mechanumMotor.hasResetOccurred() || beltMotor.hasResetOccurred() || kickMotor.hasResetOccurred()){
            mechanumMotor.optimizeBusUtilization(40);
            beltMotor.optimizeBusUtilization(40);
            kickMotor.optimizeBusUtilization(40);
            mechanumMotor.getPosition().setUpdateFrequency(40);
        }
        
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        inputs.indexerData = new IndexerIO.IndexerIOData(
            mechanumMotor.isConnected(),
            beltMotor.isConnected(),
            kickMotor.isConnected(),
            mechanumCurrent.getValueAsDouble(),
            beltCurrent.getValueAsDouble(),
            kickCurrent.getValueAsDouble(),
            mechanumVoltage.getValueAsDouble(),
            beltVoltage.getValueAsDouble(),
            kickVoltage.getValueAsDouble(),
            mechanumMotor.getVelocity().getValueAsDouble(),
            beltMotor.getVelocity().getValueAsDouble(),
            kickMotor.getVelocity().getValueAsDouble(),
            mechanumMotor.getDeviceTemp().getValueAsDouble(),
            beltMotor.getDeviceTemp().getValueAsDouble(),
            kickMotor.getDeviceTemp().getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        voltageRequest = new VoltageOut(voltage);
        mechanumMotor.setControl(voltageRequest);
        beltMotor.setControl(voltageRequest);
        kickMotor.setControl(voltageRequest);
    }

    @Override
    public void stop(){
        mechanumMotor.stopMotor(); 
        beltMotor.stopMotor();
        kickMotor.stopMotor();
    }


}
