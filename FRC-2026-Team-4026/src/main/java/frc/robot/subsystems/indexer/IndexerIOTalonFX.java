package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class IndexerIOTalonFX implements IndexerIO{
    private TalonFX leftMotor, rightMotor;
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageRequest;

    private StatusSignal<Voltage> voltageRight;
    private StatusSignal<Voltage> voltageLeft;

    public IndexerIOTalonFX(){
        leftMotor = new TalonFX(Ports.INDEXER_MOTOR_LEFT);
        rightMotor = new TalonFX(Ports.INDEXER_MOTOR_RIGHT); 

        leftMotor.setControl(new Follower(Ports.INDEXER_MOTOR_RIGHT, MotorAlignmentValue.Opposed));

        voltageLeft = leftMotor.getMotorVoltage();
        voltageRight = rightMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(40, voltageLeft, voltageRight);
    }
    @Override
    public void periodic(){
        if(leftMotor.hasResetOccurred() || rightMotor.hasResetOccurred()){
            rightMotor.optimizeBusUtilization();
            leftMotor.optimizeBusUtilization();
            rightMotor.getPosition().setUpdateFrequency(40);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        inputs.indexerData = new IndexerIO.IndexerIOData(
            leftMotor.isConnected(),
            rightMotor.isConnected(),
            voltageLeft.getValueAsDouble(),
            voltageRight.getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        voltageRequest = new VoltageOut(voltage);
        leftMotor.setControl(voltageRequest);
        rightMotor.setControl(voltageRequest);
    }

    @Override
    public void stop(){
        rightMotor.stopMotor(); 
        leftMotor.stopMotor();
    }


}
