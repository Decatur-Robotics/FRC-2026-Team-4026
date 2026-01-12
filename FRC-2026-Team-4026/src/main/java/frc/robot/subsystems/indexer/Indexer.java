package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private double voltage;
    private IndexerIO io;
    private final String inputsName;

    public Indexer(IndexerIO io) {
        this.inputsName = this.getClass().getSimpleName() + "Inputs";
        this.io = io
    }
    
}
