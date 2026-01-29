package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;
    private TalonFXConfiguration config;
    private StatusSignal<Angle> turretPosition;
    private StatusSignal<Voltage> turretVoltage;
    private StatusSignal<Current> turretSupplyCurrent;
    private MotionMagicVoltage positionRequest;
    public TurretIOTalonFX() {
        turretMotor = new TalonFX(Ports.TURRET_MOTOR_ID);

        config = new TalonFXConfiguration();
        config.Slot0 = new Slot0Configs().withKP(TurretConstants.kP)
        .withKI(TurretConstants.kI)
        .withKD(TurretConstants.kD)
        .withKS(TurretConstants.kS)
        .withKV(TurretConstants.kV)
        .withKA(TurretConstants.kA);

        turretMotor.getConfigurator().apply(config);
        turretPosition = turretMotor.getPosition();
        turretVoltage = turretMotor.getSupplyVoltage();
        turretSupplyCurrent = turretMotor.getSupplyCurrent();

        positionRequest = new MotionMagicVoltage(turretPosition.getValueAsDouble()).withEnableFOC(true);
        
        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, turretPosition, turretVoltage, turretSupplyCurrent));
        tryUntilOk(5, () -> turretMotor.optimizeBusUtilization());
        //turret is not on canivore
        PhoenixUtil.registerSignals(false, turretPosition, turretVoltage, turretSupplyCurrent);
    }

    @Override
    public void periodic(){
        //makes sure CAN stuff is right if something like a brownout happens
        if(turretMotor.hasResetOccurred()){
            turretMotor.optimizeBusUtilization();
            turretMotor.getPosition().setUpdateFrequency(40);
        }
        
    }
    
    //sets the IO data in accordance to its current readings
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretData = new TurretIOData(
            turretMotor.isConnected(),
            turretPosition.getValueAsDouble(),
            turretVoltage.getValueAsDouble(),
            turretSupplyCurrent.getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }

    //stops the motor(who could've guessed)
    @Override
    public void stop(){
        turretMotor.stopMotor();
    }

    @Override
    public void setPosition(double turretPosition){
        turretMotor.setControl(positionRequest.withPosition(turretPosition));
    }
}
