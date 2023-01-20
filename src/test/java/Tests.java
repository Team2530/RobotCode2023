import org.junit.Test;
import frc.robot.logging.*;
import static org.junit.Assert.*;

public class Tests {
    @Test 
    public void noOutputTest() {
        try {
            FRCLogger logger = new DefaultFRCLogger("", false, false);

            // This should throw an IllegalStateException
            logger.setup();

            assertTrue(false);

        } catch(IllegalStateException exception) {
        }
    }

    @Test
    public void test() {
        FRCLogger logger = new DefaultFRCLogger("[Test] ", false, true);
        logger.info("Hello world");
        logger.infoIf(logger != null, "Hello world 2");
    }
}
