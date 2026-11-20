package frc.spectrumLib.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class FirmwareCheckTest {
    private static final String[] NAMES = {"FL", "FR", "BL", "BR"};

    @Test
    void allTheSameIsFine() {
        String v = "26.1.0.0";
        assertTrue(FirmwareCheck.problems(NAMES, new String[] {v, v, v, v}).isEmpty());
    }

    @Test
    void theOddOneOutIsNamed() {
        List<String> p =
                FirmwareCheck.problems(
                        NAMES, new String[] {"26.1.0.0", "26.1.0.0", "25.3.0.0", "26.1.0.0"});
        assertEquals(List.of("BL: firmware 25.3.0.0, others 26.1.0.0"), p);
    }

    @Test
    void anUnreadDeviceIsNamed() {
        String v = "26.1.0.0";
        List<String> p =
                FirmwareCheck.problems(NAMES, new String[] {v, "unknown (RxTimeout)", v, v});
        assertEquals(List.of("FR: firmware not read (unknown (RxTimeout))"), p);
    }

    @Test
    void anEvenSplitNamesEveryDevice() {
        List<String> p =
                FirmwareCheck.problems(
                        NAMES, new String[] {"26.1.0.0", "26.1.0.0", "25.3.0.0", "25.3.0.0"});
        assertEquals(4, p.size());
    }
}
