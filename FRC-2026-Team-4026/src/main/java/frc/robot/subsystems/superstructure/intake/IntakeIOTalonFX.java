package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Ports;
import frc.robot.util.PhoenixUtil;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO{

    public TalonFX intakeMotor, deployMotor,deployFollowMotor;

    private PositionDutyCycle positionRequest;
    private DynamicMotionMagicExpoVoltage alternatePositionRequest;
    
    private VoltageOut voltageRequest;

    private CANcoder encoder;

    private double deployPosition;
    private StatusSignal<Current> deployCurrent;
    private StatusSignal<Current> deployFollowCurrent;
    
    private final MotionMagicVoltage requestVoltage = new MotionMagicVoltage(1);
 
    public TalonFXConfiguration config = new TalonFXConfiguration().withSlot0(IntakeConstants.SLOT0_CONFIGS)
    .withVoltage(new VoltageConfigs().withPeakForwardVoltage(3).withPeakReverseVoltage(3));


    public TalonFXConfiguration intakeConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT));

    
    

    public IntakeIOTalonFX(){
    var TalonFXConfigs = new TalonFXConfiguration();

    final var MotionMagicConfigs = TalonFXConfigs.MotionMagic;
    MotionMagicConfigs.MotionMagicCruiseVelocity = 5;
    MotionMagicConfigs.MotionMagicAcceleration = 10;

        intakeMotor = new TalonFX(Ports.INTAKE_MOTOR_PORT);

        deployMotor = new TalonFX(Ports.DEPLOY_FOLLOW_MOTOR_PORT);
        deployFollowMotor = new TalonFX(Ports.DEPLOY_MOTOR_PORT);
        encoder = new CANcoder(0);
        deployFollowMotor.setControl(new Follower(Ports.DEPLOY_FOLLOW_MOTOR_PORT, MotorAlignmentValue.Aligned));
        deployPosition = deployMotor.getPosition().getValueAsDouble();

        positionRequest = new PositionDutyCycle(deployPosition);
        alternatePositionRequest = new DynamicMotionMagicExpoVoltage(deployPosition, 0.05, 0.5);
        deployMotor.setPosition(encoder.getPosition().getValueAsDouble()*IntakeConstants.DEPLOY_INTAKE_POSITION/0.325);
        deployFollowMotor.setPosition(encoder.getPosition().getValueAsDouble()*IntakeConstants.DEPLOY_INTAKE_POSITION/0.325);
        deployMotor.getConfigurator().apply(config);
        deployFollowMotor.getConfigurator().apply(config);
        intakeMotor.getConfigurator().apply(intakeConfig);


        tryUntilOk(5, () -> intakeMotor.optimizeBusUtilization(40));
        tryUntilOk(5, () -> deployMotor.optimizeBusUtilization(40));
        tryUntilOk(5, () -> deployFollowMotor.optimizeBusUtilization(40));

        BaseStatusSignal.setUpdateFrequencyForAll(40.0, 
        intakeMotor.getMotorVoltage(),
        deployMotor.getMotorVoltage(),
        deployFollowMotor.getMotorVoltage(),
        intakeMotor.getSupplyCurrent(),
        deployMotor.getSupplyCurrent(),
        deployFollowMotor.getSupplyCurrent(),
        deployMotor.getPosition(),
        deployFollowMotor.getPosition(),
        intakeMotor.getDeviceTemp(),
        deployMotor.getDeviceTemp(),
        deployMotor.getDeviceTemp(),
        deployMotor.getVelocity(),
        deployMotor.getAcceleration());


        PhoenixUtil.registerSignals(false, 
            intakeMotor.getMotorVoltage(),
            deployMotor.getMotorVoltage(),
            deployFollowMotor.getMotorVoltage(),
            intakeMotor.getSupplyCurrent(),
            deployMotor.getSupplyCurrent(),
            deployFollowMotor.getSupplyCurrent(),
            deployMotor.getPosition(),
            deployFollowMotor.getPosition(),
            intakeMotor.getSupplyCurrent());
    }

    @Override
    public void updatePosition(){
        deployMotor.getPosition().getValueAsDouble();
        deployFollowMotor.getPosition().getValueAsDouble();

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
        intakeMotor.getMotorVoltage().getValueAsDouble(),
        deployMotor.getMotorVoltage().getValueAsDouble(),
        deployFollowMotor.getMotorVoltage().getValueAsDouble(),
        intakeMotor.getSupplyCurrent().getValueAsDouble(),
        deployMotor.getSupplyCurrent().getValueAsDouble(),
        deployFollowMotor.getSupplyCurrent().getValueAsDouble(),
        deployMotor.getPosition().getValueAsDouble(),
        deployFollowMotor.getPosition().getValueAsDouble(),
        intakeMotor.getDeviceTemp().getValueAsDouble(),
        deployMotor.getDeviceTemp().getValueAsDouble(),
        deployMotor.getDeviceTemp().getValueAsDouble(),
        deployMotor.getVelocity().getValueAsDouble(),
        deployMotor.getAcceleration().getValueAsDouble(),
        encoder.getPosition().getValueAsDouble()
        );
    }
    @Override
    public void setIntakeVoltage(double voltage){

        voltageRequest = new VoltageOut(voltage);

        intakeMotor.setControl(voltageRequest);

    }

    @Override
    public void setDeployPosition(double posRot){
        this.deployPosition = posRot;
        deployMotor.setControl(positionRequest.withPosition(posRot));
    }

    public void setAltDeployPosition(double posRot){
        deployMotor.setControl(alternatePositionRequest.withPosition(posRot).withVelocity(0.05));
    }

    @Override
    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    @Override
    public void setDeployVoltage(double voltage){
        voltageRequest = new VoltageOut(voltage);
        deployMotor.setVoltage(voltage);
    }

    @Override
    public void coast(){
        deployMotor.setControl(new CoastOut());
    }

    public void setSlowPosition(){
        deployMotor.setControl(requestVoltage);

    }


}
