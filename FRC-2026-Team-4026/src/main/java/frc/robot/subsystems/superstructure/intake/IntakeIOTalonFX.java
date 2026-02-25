package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO{

    public TalonFX intakeMotor, deployMotor,deployFollowMotor;

    private MotionMagicVoltage positionRequest;
    private VoltageOut voltageRequest;

    private StatusSignal<Angle> deployPosition;
    private StatusSignal<Current> deployCurrent;
    private StatusSignal<Current> intakeCurrent;
    private StatusSignal<Voltage> intakeVoltage;
    private StatusSignal<Angle> deployFollowPosition;
    private StatusSignal<Current> deployFollowCurrent;
    private StatusSignal<Voltage> deployVoltage;
    private StatusSignal<Voltage> deployFollowVoltage;

    public TalonFXConfiguration config = new TalonFXConfiguration().withSlot0(IntakeConstants.SLOT0_CONFIGS);

    public IntakeIOTalonFX(){
        intakeMotor = new TalonFX(Ports.INTAKE_MOTOR_PORT);

        deployMotor = new TalonFX(Ports.DEPLOY_MOTOR_PORT);
        deployFollowMotor = new TalonFX(Ports.DEPLOY_FOLLOW_MOTOR_PORT);
        deployFollowMotor.setControl(new Follower(Ports.DEPLOY_MOTOR_PORT, MotorAlignmentValue.Opposed));

                deployPosition = deployMotor.getPosition();
        deployFollowPosition = deployFollowMotor.getPosition();

        intakeCurrent = intakeMotor.getSupplyCurrent();

        positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        intakeVoltage = intakeMotor.getMotorVoltage();
        deployVoltage = deployMotor.getMotorVoltage();
        deployFollowVoltage = deployFollowMotor.getMotorVoltage();
        deployCurrent = deployMotor.getSupplyCurrent();
        deployFollowCurrent = deployFollowMotor.getSupplyCurrent();

        deployMotor.getConfigurator().apply(config);
        deployFollowMotor.getConfigurator().apply(config);
       BaseStatusSignal.setUpdateFrequencyForAll(40, intakeVoltage,deployVoltage);
        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, intakeVoltage,deployVoltage,deployFollowVoltage, intakeCurrent, deployCurrent,deployFollowCurrent, deployPosition,deployFollowPosition));
        tryUntilOk(5, () -> intakeMotor.optimizeBusUtilization());
        tryUntilOk(5, () -> deployMotor.optimizeBusUtilization());
        tryUntilOk(5, () -> deployFollowMotor.optimizeBusUtilization());
        deployFollowMotor.optimizeBusUtilization();
        PhoenixUtil.registerSignals(true, intakeVoltage, deployVoltage,deployFollowVoltage, intakeCurrent, deployCurrent,deployFollowCurrent, deployPosition,deployFollowPosition);

    }
    @Override
    public void periodic(){
        if (intakeMotor.hasResetOccurred()){
            intakeMotor.optimizeBusUtilization(40);
        }
        if (deployMotor.hasResetOccurred()){

            deployMotor.optimizeBusUtilization(40);
        }
        if(deployFollowMotor.hasResetOccurred()){
            deployFollowMotor.optimizeBusUtilization(40);
        }
    
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){

        inputs.intakeData = new IntakeIO.IntakeIOData(
        intakeMotor.isConnected(),
        deployMotor.isConnected(),
        deployFollowMotor.isConnected(),
        intakeVoltage.getValueAsDouble(),
        deployVoltage.getValueAsDouble(),
        deployFollowVoltage.getValueAsDouble(),
        intakeCurrent.getValueAsDouble(),
        deployCurrent.getValueAsDouble(),
        deployFollowCurrent.getValueAsDouble(),
        deployPosition.getValueAsDouble(),
        deployFollowPosition.getValueAsDouble(),
        intakeMotor.getDeviceTemp().getValueAsDouble(),
        deployMotor.getDeviceTemp().getValueAsDouble(),
        deployMotor.getDeviceTemp().getValueAsDouble()
        );
    }
    @Override
    public void setIntakeVoltage(double voltage){

        voltageRequest = new VoltageOut(voltage);

        intakeMotor.setControl(voltageRequest);

    }

    @Override
    public void setDeployPosition(double position){
        deployMotor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    @Override
    public void setDeployVoltage(double voltage){
        voltageRequest = new VoltageOut(voltage);
        deployMotor.setControl(voltageRequest);
    }


}
