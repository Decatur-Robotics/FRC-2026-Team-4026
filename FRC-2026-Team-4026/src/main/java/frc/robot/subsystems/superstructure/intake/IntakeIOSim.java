package frc.robot.subsystems.superstructure.intake;

public class IntakeIOSim implements IntakeIO{
    private final IntakeSimulation IntakeSimulation;
    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain)
        this.intakeSimulation = new IntakeSimulation.OverTheBumperIntake(null)
//add drivetrain the otb params
//acctually add all of the otb params, 
// drivetrain, width, lengthextended, side, capacity

    @Override
    public void setDeployPosition(double position) {
        if (position == IntakeConstants.DEPLOY_INTAKE_POSITION){
            IntakeSimulation.startIntake();
        }
        else
            IntakeSimulation.stopIntake();


        }

}
