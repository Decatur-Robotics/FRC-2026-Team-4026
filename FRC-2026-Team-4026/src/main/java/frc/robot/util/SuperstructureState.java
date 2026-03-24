package frc.robot.util;

public class SuperstructureState {
    public double shooterVelocity;
    public double hoodAngle;
    public double intakeDeployed;
    public double indexerVoltage;
    public double intakeVoltage;

    public SuperstructureState(double shooterVelocity, double intakeDeployed, double indexerRunning, double intakeVoltage){
        this.shooterVelocity = shooterVelocity;

        this.intakeDeployed = intakeDeployed;
        this.indexerVoltage = indexerRunning;
        this.intakeVoltage = intakeVoltage;
    }

    public SuperstructureState(double shooterVelocity, double indexerRunning, double intakeVoltage){
        this.shooterVelocity = shooterVelocity;

        this.indexerVoltage = indexerRunning;
        this.intakeVoltage = intakeVoltage;
    }
    public SuperstructureState(double intakeDeployed, double intakeVoltage){
        this.intakeDeployed = intakeDeployed;
        this.intakeVoltage = intakeVoltage;
    }

    

    public SuperstructureState copyInstance(){
        return new SuperstructureState(shooterVelocity, intakeDeployed, indexerVoltage, intakeVoltage);
    }
}
