package frc.robot.util;

public class SuperstructureState {
    public double shooterVelocity;
    public double hoodAngle;
    public double intakeDeployed;
    public double indexerVoltage;
    public double intakeVoltage;

    public SuperstructureState(double shooterVelocity, double hoodAngle, double intakeDeployed, double indexerRunning, double intakeVoltage){
        this.shooterVelocity = shooterVelocity;
        this.hoodAngle = hoodAngle;
        this.intakeDeployed = intakeDeployed;
        this.indexerVoltage = indexerRunning;
        this.intakeVoltage = intakeVoltage;
    }

    public SuperstructureState(double shooterVelocity, double hoodAngle, double indexerRunning, double intakeVoltage){
        this.shooterVelocity = shooterVelocity;
        this.hoodAngle = hoodAngle;
        this.indexerVoltage = indexerRunning;
        this.intakeVoltage = intakeVoltage;
    }

    public SuperstructureState(double shooterVelocity, double hoodAngle, double indxerRunning){
        this.shooterVelocity = shooterVelocity;
        this.hoodAngle = hoodAngle;
        this.indexerVoltage = indxerRunning; 
    }

    public SuperstructureState(double intakeDeployed, double intakeVoltage){
        this.intakeDeployed = intakeDeployed;
        this.intakeVoltage = intakeVoltage;
    }

    public SuperstructureState copyInstatnce(){
        return new SuperstructureState(shooterVelocity, hoodAngle, intakeDeployed, indexerVoltage, intakeVoltage);
    }
}
