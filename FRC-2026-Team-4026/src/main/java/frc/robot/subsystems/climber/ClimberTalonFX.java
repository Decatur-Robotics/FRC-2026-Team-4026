package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;

public class ClimberTalonFX implements ClimberIO {

	private final TalonFX motor;
	private TalonFXConfiguration config;

    private StatusSignal<Angle> position;
    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> current;
    private MotionMagicVoltage positionRequest;


	public ClimberTalonFX() {
        
		this.motor = new TalonFX(Ports.CLIMBER_MOTOR_LEFT);

		this.config = new TalonFXConfiguration();

		config.Slot0 = new Slot0Configs().withKP(ClimberConstants.kP).withKI(ClimberConstants.kI).withKD(ClimberConstants.kD).withKS(ClimberConstants.kS).withKV(ClimberConstants.kV).withKA(ClimberConstants.kA);
		motor.getConfigurator().apply(config);

        position = motor.getPosition();
        voltage = motor.getSupplyVoltage();
        current = motor.getSupplyCurrent();

        positionRequest = new MotionMagicVoltage(position.getValueAsDouble()).withEnableFOC(true);
        
	}

	@Override
    public void periodic() {
        if(motor.hasResetOccurred()){
            motor.optimizeBusUtilization();
            motor.getPosition().setUpdateFrequency(40);
        }
        
    }
    
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberData = new ClimberIOData(
            motor.isConnected(),
            position.getValueAsDouble(),
            voltage.getValueAsDouble(),
            current.getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public void stop(){
        motor.stopMotor();
    }

    @Override
    public void setPosition(double position){
        motor.setControl(positionRequest.withPosition(position));
    }

}
