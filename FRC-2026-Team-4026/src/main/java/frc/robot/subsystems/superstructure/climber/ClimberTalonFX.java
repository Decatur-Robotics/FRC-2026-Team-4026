package frc.robot.subsystems.superstructure.climber;

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

public class ClimberTalonFX implements ClimberIO {

    public boolean inverted;
    private int following = -1;

	private final TalonFX motor, followMotor;
	private TalonFXConfiguration config;

    private StatusSignal<Angle> position;
    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> current;
    private MotionMagicVoltage positionRequest;


	public ClimberTalonFX(int port, int followPort) {
        
		this.motor = new TalonFX(port);
		this.followMotor = new TalonFX(followPort);

        followMotor.setControl(new Follower(following, MotorAlignmentValue.Opposed)); // todo ask cad if this is right

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
            current.getValueAsDouble()
        );
    }

    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(inverted ? -voltage : voltage);
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
