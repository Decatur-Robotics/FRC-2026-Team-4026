package frc.robot.subsystems.superstructure.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO{
    private TalonFX mecanumMotor, beltMotor, kickMotor;
    private Timer indexerTimer;
    
    private TalonFXConfiguration config = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT));

    private VoltageOut voltageRequest;

    public IndexerIOTalonFX(){
        indexerTimer = new Timer();
        mecanumMotor = new TalonFX(Ports.INDEXER_MOTOR_MECANUM);
         beltMotor = new TalonFX(Ports.INDEXER_MOTOR_BELT); 
        kickMotor = new TalonFX(Ports.INDEXER_MOTOR_KICK);

        mecanumMotor.getConfigurator().apply(config);
        beltMotor.getConfigurator().apply(config);
        kickMotor.getConfigurator().apply(config);

        beltMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECANUM, MotorAlignmentValue.Opposed));
        kickMotor.setControl(new Follower(Ports.INDEXER_MOTOR_MECANUM, MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(40,
            mecanumMotor.getSupplyCurrent(),
            beltMotor.getSupplyCurrent(),
            kickMotor.getSupplyCurrent(),
            mecanumMotor.getMotorVoltage(),
            beltMotor.getMotorVoltage(),
            kickMotor.getMotorVoltage(),
            mecanumMotor.getVelocity(),
            beltMotor.getVelocity(),
            kickMotor.getVelocity(),
            mecanumMotor.getDeviceTemp(),
            beltMotor.getDeviceTemp(),
            kickMotor.getDeviceTemp());

        PhoenixUtil.registerSignals(false,   mecanumMotor.getSupplyCurrent(),
            beltMotor.getSupplyCurrent(),
            kickMotor.getSupplyCurrent(),
            mecanumMotor.getMotorVoltage(),
            beltMotor.getMotorVoltage(),
            kickMotor.getMotorVoltage(),
            mecanumMotor.getVelocity(),
            beltMotor.getVelocity(),
            kickMotor.getVelocity(),
            mecanumMotor.getDeviceTemp(),
            beltMotor.getDeviceTemp(),
            kickMotor.getDeviceTemp());
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
               mecanumMotor.getSupplyCurrent().getValueAsDouble(),
            beltMotor.getSupplyCurrent().getValueAsDouble(),
            kickMotor.getSupplyCurrent().getValueAsDouble(),
            mecanumMotor.getMotorVoltage().getValueAsDouble(),
            beltMotor.getMotorVoltage().getValueAsDouble(),
            kickMotor.getMotorVoltage().getValueAsDouble(),
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
    if (indexerTimer.get() > 0.25){
        voltageRequest = new VoltageOut(voltage);

        beltMotor.setControl(voltageRequest);
        kickMotor.setControl(voltageRequest);

        voltageRequest = new VoltageOut(voltage*-1);
        mecanumMotor.setControl(voltageRequest);
        }
    }


    @Override
    public void stop(){
        mecanumMotor.stopMotor(); 
        beltMotor.stopMotor();
        kickMotor.stopMotor();
    }
    @Override
    public void resetTimer(){
        indexerTimer.reset();
    }

}
