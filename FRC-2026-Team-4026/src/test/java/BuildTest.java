import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.indexer.IndexerIO;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;

class BuildTest {
    @Test
    void buildTest() {
        // This test will fail if the code does not compile
    }
    
    @Test
    void testIndexerIOInstantiation() {
        assertDoesNotThrow(() -> {
            IndexerIO indexerIO = new IndexerIOSim();
            assertNotNull(indexerIO);
        });
    }
    
    @Test
    void testIndexerInstantiation() {
        assertDoesNotThrow(() -> {
            IndexerIO indexerIO = new IndexerIOSim();
            Indexer indexer = new Indexer(indexerIO);
            assertNotNull(indexer);
        });
    }
    
    @Test
    void testRobotContainerInstantiation() {
        // Test that RobotContainer can be instantiated (main entry point)
        assertDoesNotThrow(() -> {
            RobotContainer container = new RobotContainer();
            assertNotNull(container);
        });
    }
}