package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class BatteryNamesTest {
    @Test
    void parsesListSkippingCommentsAndBadLines() {
        BatteryNames names =
                BatteryNames.parse(
                        "# UID,name\n\nd7a89b9e , Bat 1\r\nnot a line\n04A1B2C3D4E5F6,Bat 2\n");
        assertEquals(2, names.size());
        assertEquals("Bat 1", names.name("D7A89B9E"));
        assertEquals("Bat 2", names.name("04a1b2c3d4e5f6"));
        assertEquals("", names.name("FFFFFFFF"));
    }

    @Test
    void missingFileIsEmpty() {
        assertEquals(0, BatteryNames.load(Path.of("no-such-dir", "batteries.csv")).size());
    }

    @Test
    void deployedListLoads() {
        BatteryNames names = BatteryNames.load(Path.of("src/main/deploy/batteries.csv"));
        assertEquals("test-tag-1", names.name("D7A89B9E"));
    }
}
