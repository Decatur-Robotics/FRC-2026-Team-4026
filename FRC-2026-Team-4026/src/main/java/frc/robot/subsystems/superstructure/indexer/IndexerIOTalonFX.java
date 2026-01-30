package frc.robot.subsystems.superstructure.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;

public class IndexerIOTalonFX implements IndexerIO{
    private TalonFX leftMotor, rightMotor;
    
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageRequest;

    private StatusSignal<Voltage> voltageRight;
    private StatusSignal<Voltage> voltageLeft;
    private StatusSignal<Current> leftCurrent;
    private StatusSignal<Current> rightCurrent;

    public IndexerIOTalonFX(){
        leftMotor = new TalonFX(Ports.INDEXER_MOTOR_LEFT);
        rightMotor = new TalonFX(Ports.INDEXER_MOTOR_RIGHT); 

        voltageLeft = leftMotor.getMotorVoltage();
        voltageRight = rightMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        rightCurrent = rightMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(40, voltageLeft, voltageRight);
    }

    @Override
    public void periodic(){
        if(leftMotor.hasResetOccurred() || rightMotor.hasResetOccurred()){
            rightMotor.optimizeBusUtilization(40);
            leftMotor.optimizeBusUtilization(40);
            rightMotor.getPosition().setUpdateFrequency(40);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs){
        inputs.indexerData = new IndexerIO.IndexerIOData(
            leftMotor.isConnected(),
            rightMotor.isConnected(),
            leftCurrent.getValueAsDouble(),
            rightCurrent.getValueAsDouble(),
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
