package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.util.PhoenixUtil;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO{

    public TalonFX intakeMotor, deployMotor;

    private MotionMagicVoltage positionRequest;
    private VoltageOut voltageRequest;

    private StatusSignal<Angle> deployPosition;
    private StatusSignal<Current> deployCurrent;
    private StatusSignal<Current> intakeCurrent;
    private StatusSignal<Voltage> intakeVoltage;
    private StatusSignal<Voltage> deployVoltage;

    public TalonFXConfiguration config = new TalonFXConfiguration();

    public IntakeIOTalonFX(){
        config.Slot0 = new Slot0Configs()
        .withKA(IntakeConstants.kA)
        .withKI(IntakeConstants.kI)
        .withKD(IntakeConstants.kD)
        .withKS(IntakeConstants.kS)
        .withKV(IntakeConstants.kV)
        .withKA(IntakeConstants.kA);

        deployPosition = deployMotor.getPosition();
        intakeCurrent = intakeMotor.getSupplyCurrent();

        positionRequest = new MotionMagicVoltage(deployPosition.getValueAsDouble()).withEnableFOC(true);
        intakeMotor = new TalonFX(Ports.INTAKE_MOTOR_PORT);
        deployMotor = new TalonFX(Ports.DEPLOY_MOTOR_PORT);
        
        intakeVoltage = intakeMotor.getMotorVoltage();
        deployVoltage = deployMotor.getMotorVoltage();
        deployMotor.getConfigurator().apply(config);
        BaseStatusSignal.setUpdateFrequencyForAll(40, intakeVoltage,deployVoltage);
        tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(40.0, intakeVoltage,deployVoltage, intakeCurrent, deployCurrent, deployPosition));
        tryUntilOk(5, () -> intakeMotor.optimizeBusUtilization());
        tryUntilOk(5, () -> deployMotor.optimizeBusUtilization());
        PhoenixUtil.registerSignals(true, intakeVoltage, deployVoltage, intakeCurrent, deployCurrent, deployPosition);

    }
    @Override
    public void periodic(){
        if (intakeMotor.hasResetOccurred()){
            intakeMotor.optimizeBusUtilization(40);
        }
        if (deployMotor.hasResetOccurred()){

            deployMotor.optimizeBusUtilization(40);
        }
    
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){

        inputs.intakeData = new IntakeIO.IntakeIOData(
        intakeMotor.isConnected(),
        deployMotor.isConnected(),
        intakeVoltage.getValueAsDouble(),
        deployVoltage.getValueAsDouble(),
        intakeCurrent.getValueAsDouble(),
        deployCurrent.getValueAsDouble(),
        deployPosition.getValueAsDouble()
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



}
