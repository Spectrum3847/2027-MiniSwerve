package frc.spectrumLib.robotsettings;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class RobotSettingsInputsTest {
    @Test
    void replayRestoresTheLoggedValuesAndSources() {
        var robot = new SettingStore();
        var g = new SettingGroup("g", robot::register);
        DoubleSetting n = g.number("n", 1, "");
        IntSetting i = g.integer("i", 1, "");
        BooleanSetting b = g.bool("b", true);
        StringSetting s = g.string("s", "x");
        robot.restore(n, 2.5, SettingSource.LOCAL);
        robot.restore(i, 7, SettingSource.LIVE);
        robot.restore(b, false, SettingSource.DEPLOY);
        robot.restore(s, "y", SettingSource.ROBOT);

        LogTable table = new LogTable(0);
        new RobotSettingsInputs(robot).toLog(table);

        var replay = new SettingStore();
        var g2 = new SettingGroup("g", replay::register);
        DoubleSetting n2 = g2.number("n", 1, "");
        IntSetting i2 = g2.integer("i", 1, "");
        BooleanSetting b2 = g2.bool("b", true);
        StringSetting s2 = g2.string("s", "x");
        DoubleSetting newer = g2.number("newer", 9, "");
        new RobotSettingsInputs(replay).fromLog(table);

        assertEquals(2.5, n2.get());
        assertEquals(SettingSource.LOCAL, n2.source());
        assertEquals(7, i2.get());
        assertEquals(SettingSource.LIVE, i2.source());
        assertEquals(false, b2.get());
        assertEquals("y", s2.get());
        assertEquals(SettingSource.ROBOT, s2.source());
        assertEquals(9.0, newer.get(), "not in the log: keeps its own value");
        assertEquals(SettingSource.DEFAULT, newer.source());
    }
}
