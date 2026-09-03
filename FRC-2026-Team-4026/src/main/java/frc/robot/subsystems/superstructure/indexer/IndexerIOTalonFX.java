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

    private Indexer indexer;

    private TalonFXConfiguration config = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT));

    private TalonFXConfiguration kickConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));

    private VoltageOut voltageRequest;

    public IndexerIOTalonFX(Indexer indexer){

        this.indexer = indexer;

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
        if(indexer.getMecanumCurrent() > IndexerConstants.INDEXER_CURRENT_LIMIT && indexer.getMecanumVelocity() == 0){
            setVoltage(-6);
            Timer.delay(0.7);
            setVoltage(6);
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

        voltageRequest = new VoltageOut(voltage);

        beltMotor.setControl(voltageRequest);
        kickMotor.setControl(voltageRequest);

        voltageRequest = new VoltageOut(voltage*-1);
        mecanumMotor.setControl(voltageRequest);
        
    }


    @Override
    public void stop(){
        mecanumMotor.stopMotor(); 
        beltMotor.stopMotor();
        kickMotor.stopMotor();
    }



}
