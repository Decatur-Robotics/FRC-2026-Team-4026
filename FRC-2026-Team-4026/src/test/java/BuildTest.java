import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIO;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;

class BuildTest {
    IndexerIO indexerIO;
    Indexer indexer;
    RobotContainer container;
    @Test
    void testIndexerIOInstantiation() {
        assertDoesNotThrow(() -> {
            indexerIO = new IndexerIOSim();
        });
        assertNotNull(indexerIO);
    }
    

    @Test
    void testIndexerInstantiation() {
        assertDoesNotThrow(() -> {
            indexer = new Indexer(indexerIO);
        });
        assertNotNull(indexer);
    }
    
    @Test
    void testRobotContainerInstantiation() {
        // Test that RobotContainer can be instantiated (main entry point)
        assertDoesNotThrow(() -> {
            container = new RobotContainer();
        });
        assertNotNull(container);
    }
}