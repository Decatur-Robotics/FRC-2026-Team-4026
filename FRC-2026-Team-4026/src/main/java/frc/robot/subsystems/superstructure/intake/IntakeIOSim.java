package frc.robot.subsystems.superstructure.intake;

import java.security.cert.X509CRL;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
//add params please!
public class IntakeIOSim implements IntakeIO{

    private final DCMotorSim intakeSim;
    private final SingleJointedArmSim deploySim;
    private final ProfiledPIDController controller = new ProfiledPIDController(
    IntakeConstants.kP, 
    IntakeConstants.kI, 
    IntakeConstants.kD,
    new TrapezoidProfile.Constraints(0, 0));
    private double pidOutput;
    private final IntakeSimulation intakeSimulation;
        //make encoder! add the encoder channels this goes in overall constants file?
        private final Encoder encoder = new Encoder(0, 1);
        private final EncoderSim encoderSim = new EncoderSim(encoder);

        public IntakeIOSim(AbstractDriveTrainSimulation drivetrain){
            encoder.setDistancePerPulse(IntakeConstants.ENCODER_DIST_PER_PULSE);
    
            intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.01,0.01), 
            DCMotor.getKrakenX44(1));
            //need params for singlejoitned arm sim

            deploySim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX44(2),

            IntakeConstants.DEPLOY_MOI,
            17.31),
            DCMotor.getKrakenX44(1), 
            17.3, 
            IntakeConstants.DEPLOY_LENGTH, 
            (IntakeConstants.DEPLOY_MIN_ANGLE*3.14)/180, 
            (IntakeConstants.DEPLOY_MAX_ANGLE*3.14)/180,
            true, 
            (IntakeConstants.STORED_INTAKE_POSITION*3.14)/180);
    

            
            this.intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", 
            drivetrain,
            IntakeConstants.INTAKE_WIDTH,
            IntakeConstants.INTAKE_EXTEND_LENGTH, 
            IntakeConstants.INTAKE_SIDE,
            IntakeConstants.INTAKE_CAPACITY);

            //I don't think getGamePieceContactListener should be here not sure thbough
            intakeSim.update(0);
            deploySim.update(0);
    }

    @Override
    public void periodic(){

        encoderSim.setDistance(deploySim.getAngleRads());

    }

    @Override
    public void setDeployPosition(double position) {

        deploySim.setState(position, 0);

        // if (position == IntakeConstants.DEPLOY_INTAKE_POSITION){

        //     controller.setGoal(position);
        //     double pidOutput = controller.calculate(encoderSim.getDistance(),
        //     Units.degreesToRadians(IntakeConstants.DEPLOY_INTAKE_POSITION));
        //     deploySim.setInputVoltage(pidOutput);

        //     intakeSimulation.startIntake();
        // }
        // else
        // {
        //     controller.setGoal(position);
        //     double pidOutput = controller.calculate(encoderSim.getDistance(),
        //     Units.degreesToRadians(IntakeConstants.STORED_INTAKE_POSITION));
        //     deploySim.setInputVoltage(pidOutput);

        //     intakeSimulation.stopIntake();
        // }
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.intakeData = new IntakeIOData(
        true,
        true,
        true,
        intakeSim.getInputVoltage(),
        pidOutput,
        pidOutput,
        intakeSim.getCurrentDrawAmps(),
        deploySim.getCurrentDrawAmps(),
        deploySim.getCurrentDrawAmps(),
        deploySim.getAngleRads(),
        deploySim.getAngleRads(),
        0.0,
        0.0,
        0.0
        );
    }

    @Override
    public void setIntakeVoltage(double voltage){

        intakeSim.setInputVoltage(voltage);
        intakeSimulation.startIntake();
    }

    @Override
    public void stopIntake(){
        //is this right?>
        intakeSim.setInputVoltage(0);
    }

    public void zeroCommand(double position, double voltage){

        intakeSim.setInputVoltage(voltage);
        setDeployPosition(position);
    
    }

}

