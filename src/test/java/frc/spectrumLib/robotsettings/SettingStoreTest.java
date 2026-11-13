package frc.spectrumLib.robotsettings;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class SettingStoreTest {
    @TempDir Path tmp;

    private Path deploy;
    private Path local;
    private SettingStore store;
    private SettingGroup cams;

    @BeforeEach
    void setUp() throws IOException {
        deploy = Files.createDirectories(tmp.resolve("deploy/settings"));
        local = tmp.resolve("local");
        store = new SettingStore();
        cams = new SettingGroup("cams", store::register);
    }

    private void file(Path path, String json) throws IOException {
        Files.createDirectories(path.getParent());
        Files.writeString(path, json);
    }

    private Path deployFile() {
        return deploy.resolve("cams.json");
    }

    private Path robotFile(String robot) {
        return deploy.resolve("robots").resolve(robot).resolve("cams.json");
    }

    /** This robot's local folder: {@code <local>/<robot>/}. */
    private Path robotLocal() {
        return local.resolve("FM2026");
    }

    private Path localFile() {
        return robotLocal().resolve("cams.json");
    }

    private void configure() {
        store.configure(new SettingStore.Locations(deploy, "FM2026", local));
    }

    private boolean hasProblem(SettingStore.Level level, String text) {
        return store.problems().values().stream()
                .anyMatch(p -> p.level() == level && p.text().contains(text));
    }

    // ── Precedence ──────────────────────────────────────────────────────────

    @Test
    void defaultWhenNoFiles() {
        configure();
        DoubleSetting p = cams.number("rate", 2.0, "s");
        assertEquals(2.0, p.get());
        assertEquals(SettingSource.DEFAULT, p.source());
        assertTrue(store.problems().isEmpty(), "missing files are not a problem");
    }

    @Test
    void eachLayerBeatsTheOneBelow() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1, \"c\": 1}");
        file(robotFile("FM2026"), "{\"b\": 2, \"c\": 2}");
        file(localFile(), "{\"c\": 3}");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        DoubleSetting b = cams.number("b", 0, "");
        DoubleSetting c = cams.number("c", 0, "");
        DoubleSetting d = cams.number("d", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(SettingSource.DEPLOY, a.source());
        assertEquals(2.0, b.get());
        assertEquals(SettingSource.ROBOT, b.source());
        assertEquals(3.0, c.get());
        assertEquals(SettingSource.LOCAL, c.source());
        assertEquals(0.0, d.get());
        assertEquals(SettingSource.DEFAULT, d.source());
    }

    @Test
    void anotherRobotsFileIsIgnored() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(robotFile("PM2026"), "{\"a\": 9}");
        configure();
        assertEquals(1.0, cams.number("a", 0, "").get());
    }

    @Test
    void declaredBeforeConfigureIsResolvedAgain() throws IOException {
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(SettingSource.DEFAULT, a.source());
        file(deployFile(), "{\"a\": 1}");
        configure();
        assertEquals(1.0, a.get());
        assertEquals(SettingSource.DEPLOY, a.source());
    }

    @Test
    void valueMayBeWrappedInAnObject() throws IOException {
        file(localFile(), "{\"a\": {\"value\": 4, \"replaced\": 1, \"savedAt\": \"x\"}}");
        configure();
        assertEquals(4.0, cams.number("a", 0, "").get());
    }

    @Test
    void allTypes() throws IOException {
        file(deployFile(), "{\"n\": 1.5, \"i\": 3, \"i2\": 4.0, \"b\": false, \"s\": \"hi\"}");
        configure();
        assertEquals(1.5, cams.number("n", 0, "").get());
        assertEquals(3, cams.integer("i", 0, "").get());
        assertEquals(4, cams.integer("i2", 0, "").get());
        assertEquals(false, cams.bool("b", true).get());
        assertEquals("hi", cams.string("s", "").get());
    }

    // ── Bad files and values ────────────────────────────────────────────────

    @Test
    void badDeployFileFallsBackToDefaultWithAnError() throws IOException {
        file(deployFile(), "{\"a\": 1,,}");
        configure();
        DoubleSetting a = cams.number("a", 7, "");
        assertEquals(7.0, a.get());
        assertEquals(SettingSource.DEFAULT, a.source());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "settings/cams.json is unreadable"));
    }

    @Test
    void badLocalFileFallsBackToTheRobotAndDeployLayers() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1}");
        file(robotFile("FM2026"), "{\"b\": 2}");
        file(localFile(), "not json at all");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        DoubleSetting b = cams.number("b", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(2.0, b.get());
        assertEquals(SettingSource.ROBOT, b.source());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "unreadable"));
    }

    @Test
    void badRobotFileStillLetsTheLocalFileApply() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(robotFile("FM2026"), "[1, 2]");
        file(localFile(), "{\"a\": 3}");
        configure();
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "not a JSON object"));
    }

    @Test
    void emptyFileIsUnreadable() throws IOException {
        file(deployFile(), "");
        configure();
        assertEquals(5.0, cams.number("a", 5, "").get());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "unreadable"));
    }

    @Test
    void duplicateKeyMakesTheFileUnreadable() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(localFile(), "{\"a\": 3, \"a\": 4}");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(1.0, a.get(), "neither duplicate is used");
        assertEquals(SettingSource.DEPLOY, a.source());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "unreadable (Duplicate field 'a'"));
    }

    @Test
    void duplicateKeyInsideAWrappedValueMakesTheFileUnreadable() throws IOException {
        file(localFile(), "{\"a\": {\"value\": 3, \"value\": 4}}");
        configure();
        assertEquals(0.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "Duplicate field 'value'"));
    }

    @Test
    void checkFileReportsADuplicateKey() throws IOException {
        cams.number("a", 0, "");
        file(deployFile(), "{\"a\": 1, \"a\": 1}");
        var problems = store.checkFile(deployFile());
        assertEquals(1, problems.size());
        assertTrue(problems.get(0).contains("Duplicate field 'a'"), problems.get(0));
    }

    @Test
    void wrongTypeOrOutOfBoundsValueIsSkipped() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1, \"i\": 2}");
        file(robotFile("FM2026"), "{\"a\": \"fast\", \"b\": 500, \"i\": 2.5}");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        DoubleSetting b = cams.number("b", 0, "", 0, 10);
        IntSetting i = cams.integer("i", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(SettingSource.DEPLOY, a.source());
        assertEquals(1.0, b.get());
        assertEquals(2, i.get());
        assertTrue(hasProblem(SettingStore.Level.WARNING, "a expected a number"));
        assertTrue(hasProblem(SettingStore.Level.WARNING, "b 500.0 is outside [0.0, 10.0]"));
        assertTrue(hasProblem(SettingStore.Level.WARNING, "i expected a whole number"));
    }

    @Test
    void refusedInFileNamesTheSettingsAFileCouldNotSet() throws IOException {
        file(deployFile(), "{\"a\": 1, \"ok\": 1}");
        file(robotFile("FM2026"), "{\"a\": 500}");
        configure();
        DoubleSetting a = cams.number("a", 0, "", 0, 10);
        DoubleSetting ok = cams.number("ok", 0, "", 0, 10);
        DoubleSetting absent = cams.number("absent", 0, "");
        assertTrue(store.refusedInFile(a));
        assertFalse(store.refusedInFile(ok));
        assertFalse(store.refusedInFile(absent));
        assertEquals(1.0, a.get(), "unchanged: the accessor only reads");

        SettingStore other = new SettingStore();
        SettingGroup g = new SettingGroup("cams", other::register);
        file(localFile(), "not json");
        other.configure(new SettingStore.Locations(deploy, "FM2026", local));
        assertTrue(other.refusedInFile(g.number("ok", 0, "", 0, 10)), "unreadable file");
    }

    @Test
    void unknownKeyIsInfoUntilDeclared() throws IOException {
        file(deployFile(), "{\"a\": 1, \"typo\": 2}");
        configure();
        cams.number("a", 0, "");
        assertTrue(hasProblem(SettingStore.Level.INFO, "has keys no code declares: typo"));
        cams.number("typo", 0, "");
        assertFalse(hasProblem(SettingStore.Level.INFO, "no code declares"));
    }

    @Test
    void localValuesAreCounted() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configure();
        cams.number("a", 0, "");
        assertTrue(
                hasProblem(SettingStore.Level.INFO, "1 value from this robot's local overrides"));
    }

    @Test
    void badDeclarationsThrow() {
        cams.number("a", 0, "");
        assertThrows(IllegalArgumentException.class, () -> cams.number("a", 0, ""));
        assertThrows(IllegalArgumentException.class, () -> cams.number("b", 11, "", 0, 10));
        assertThrows(IllegalArgumentException.class, () -> cams.number("c", Double.NaN, ""));
        assertThrows(IllegalArgumentException.class, () -> cams.number("has space", 0, ""));
    }

    @Test
    void checkFileReportsWithoutApplying() throws IOException {
        DoubleSetting a = cams.number("a", 0, "", 0, 10);
        file(deployFile(), "{\"a\": 20, \"typo\": 1}");
        var problems = store.checkFile(deployFile());
        assertEquals(2, problems.size());
        assertEquals(0.0, a.get());
    }

    // ── Live ────────────────────────────────────────────────────────────────

    /** A transport whose dashboard values the test sets. */
    private static final class FakeTransport implements SettingTransport {
        final Map<String, Object> values = new HashMap<>();

        @Override
        public LiveValue publish(Setting<?> setting) {
            String key = setting.path();
            values.put(key, setting.get());
            return LiveValue.of(() -> values.get(key), v -> values.put(key, v));
        }

        @Override
        public LiveValue button(String name) {
            values.put(name, false);
            return LiveValue.of(() -> values.get(name), v -> values.put(name, v));
        }
    }

    @Test
    void liveChangeAppliesOnlyWhileDisabled() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(3.0, t.values.get("cams/a"), "published with the resolved value");

        t.values.put("cams/a", 4.0);
        store.applyLive(false);
        assertEquals(3.0, a.get(), "not while enabled");
        assertTrue(store.pendingLive().contains("cams/a"));

        int version = a.version();
        store.applyLive(true);
        assertEquals(4.0, a.get());
        assertEquals(SettingSource.LIVE, a.source(), "live beats the local file");
        assertTrue(a.version() > version);
        assertTrue(store.pendingLive().isEmpty());
    }

    @Test
    void liveChangesAreAllowedOnlyDisabledOffTheField() {
        assertTrue(RobotSettings.liveChangesAllowed(true, false), "disabled in the pit");
        assertFalse(
                RobotSettings.liveChangesAllowed(true, true), "disabled between auto and teleop");
        assertFalse(RobotSettings.liveChangesAllowed(false, false), "enabled in the pit");
        assertFalse(RobotSettings.liveChangesAllowed(false, true), "enabled on the field");
    }

    @Test
    void changeRequestedDuringAMatchWaitsForThePit() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleSetting a = cams.number("a", 1, "");

        t.values.put("cams/a", 4.0);
        store.applyLive(RobotSettings.liveChangesAllowed(false, true)); // auto
        store.applyLive(RobotSettings.liveChangesAllowed(true, true)); // auto-to-teleop gap
        store.applyLive(RobotSettings.liveChangesAllowed(false, true)); // teleop
        store.applyLive(
                RobotSettings.liveChangesAllowed(true, true)); // after the match, field attached
        assertEquals(1.0, a.get(), "never applied with the field attached");
        assertEquals(SettingSource.DEFAULT, a.source());
        assertTrue(store.pendingLive().contains("cams/a"));

        store.applyLive(RobotSettings.liveChangesAllowed(true, false)); // back in the pit
        assertEquals(4.0, a.get());
        assertEquals(SettingSource.LIVE, a.source());
        assertTrue(store.pendingLive().isEmpty());
    }

    @Test
    void liveChangeBackBeforeDisableIsDropped() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleSetting a = cams.number("a", 1, "");
        t.values.put("cams/a", 4.0);
        store.applyLive(false);
        t.values.put("cams/a", 1.0);
        store.applyLive(false);
        assertTrue(store.pendingLive().isEmpty());
        store.applyLive(true);
        assertEquals(SettingSource.DEFAULT, a.source());
    }

    @Test
    void badLiveValueIsRefusedAndUndone() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleSetting a = cams.number("a", 1, "", 0, 10);
        IntSetting i = cams.integer("i", 1, "");
        t.values.put("cams/a", 99.0);
        t.values.put("cams/i", 2.5);
        store.applyLive(true);
        assertEquals(1.0, a.get());
        assertEquals(1, i.get());
        assertEquals(1.0, t.values.get("cams/a"), "dashboard shows the kept value");
        assertTrue(hasProblem(SettingStore.Level.WARNING, "cams/a refused"));

        t.values.put("cams/a", 5.0);
        store.applyLive(true);
        assertEquals(5.0, a.get());
        assertFalse(hasProblem(SettingStore.Level.WARNING, "cams/a refused"));
    }

    @Test
    void integerFromNtDouble() {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        IntSetting i = cams.integer("i", 1, "");
        t.values.put("cams/i", 3.0);
        store.applyLive(true);
        assertEquals(3, i.get());
    }

    // ── Save ────────────────────────────────────────────────────────────────

    @Test
    void saveWritesOnlyChangedValuesAndReloadsAsLocal() throws IOException {
        file(deployFile(), "{\"a\": 1, \"b\": 1}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        DoubleSetting a = cams.number("a", 0, "");
        cams.number("b", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);

        SettingStore.SaveJob job = store.prepareSave("2026-10-23T12:00:00Z");
        assertEquals(java.util.Map.of("cams/a", 2.0), job.values());
        SettingStore.write(job);

        ObjectNode saved = SettingStore.parseObject(localFile());
        assertEquals(2.0, saved.get("a").get("value").doubleValue());
        assertEquals(1.0, saved.get("a").get("replaced").doubleValue());
        assertEquals("2026-10-23T12:00:00Z", saved.get("a").get("savedAt").textValue());
        assertFalse(saved.has("b"));
        assertFalse(Files.exists(robotLocal().resolve("cams.json.tmp")));

        var next = new SettingStore();
        next.configure(new SettingStore.Locations(deploy, "FM2026", local));
        var again = new SettingGroup("cams", next::register);
        DoubleSetting a2 = again.number("a", 0, "");
        assertEquals(a.get(), a2.get());
        assertEquals(SettingSource.LOCAL, a2.source());
    }

    @Test
    void aSecondSaveCanUndoTheFirst() throws IOException {
        // No local file at boot. Tune a to 2 and save; tune it back to 0 and save again: the
        // file from the first save must not keep 2 for the next boot.
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        SettingStore.SaveJob first = store.prepareSave("first");
        SettingStore.write(first);
        store.markSaved(first);
        assertTrue(Files.readString(localFile()).contains("\"value\""));
        t.values.put("cams/a", 0.0);
        store.applyLive(true);
        SettingStore.SaveJob second = store.prepareSave("second");
        SettingStore.write(second);
        store.markSaved(second);
        assertFalse(Files.readString(localFile()).contains("\"value\""));
    }

    @Test
    void savedLiveValuesStopCountingAsUnsaved() throws IOException {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        assertTrue(hasProblem(SettingStore.Level.INFO, "1 value tuned live and not saved"));
        SettingStore.SaveJob job = store.prepareSave("now");
        SettingStore.write(job);
        store.markSaved(job);
        assertFalse(hasProblem(SettingStore.Level.INFO, "not saved"));
        assertTrue(
                hasProblem(SettingStore.Level.INFO, "1 value from this robot's local overrides"));
        t.values.put("cams/a", 3.0);
        store.applyLive(true);
        assertTrue(hasProblem(SettingStore.Level.INFO, "1 value tuned live and not saved"));
    }

    @Test
    void saveKeepsLocalEntriesAndUndeclaredKeys() throws IOException {
        file(localFile(), "{\"a\": {\"value\": 3, \"savedAt\": \"old\"}, \"gone\": 5}");
        configure();
        cams.number("a", 0, "");
        SettingStore.write(store.prepareSave("new"));
        ObjectNode saved = SettingStore.parseObject(localFile());
        assertEquals("old", saved.get("a").get("savedAt").textValue());
        assertEquals(5, saved.get("gone").intValue());
    }

    @Test
    void valueSetBackToTheGitValueLeavesTheLocalFile() throws IOException {
        file(deployFile(), "{\"a\": 1}");
        file(localFile(), "{\"a\": 3}");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 1.0);
        store.applyLive(true);
        SettingStore.write(store.prepareSave("now"));
        assertTrue(SettingStore.parseObject(localFile()).isEmpty());
    }

    @Test
    void saveMovesAnUnreadableLocalFileAside() throws IOException {
        file(localFile(), "{broken");
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        SettingStore.write(store.prepareSave("now"));
        assertEquals("{broken", Files.readString(robotLocal().resolve("cams.json.bad")));
        assertEquals(
                2.0, SettingStore.parseObject(localFile()).get("a").get("value").doubleValue());
    }

    @Test
    void staleTempFromAnInterruptedSaveIsDeletedAtBootAndNeverRead() throws IOException {
        file(localFile(), "{\"a\": 3}");
        file(robotLocal().resolve("cams.json.tmp"), "{\"a\": ");
        file(local.resolve("cams.json.tmp"), "{\"a\": ");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(3.0, a.get(), "the whole old file, not the half-written one");
        assertFalse(Files.exists(robotLocal().resolve("cams.json.tmp")));
        assertFalse(Files.exists(local.resolve("cams.json.tmp")), "the old flat folder's too");
        assertTrue(store.problems().keySet().stream().noneMatch(k -> k.startsWith("file:")));
    }

    @Test
    void saveOverAStaleTempWritesTheWholeFile() throws IOException {
        configure();
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        SettingStore.SaveJob job = store.prepareSave("now");
        file(robotLocal().resolve("cams.json.tmp"), "x".repeat(10_000));
        SettingStore.write(job);
        assertEquals(
                2.0, SettingStore.parseObject(localFile()).get("a").get("value").doubleValue());
        assertFalse(Files.exists(robotLocal().resolve("cams.json.tmp")));
    }

    @Test
    void folderFlushIsBestEffort() throws IOException {
        Files.createDirectories(local);
        boolean flushed = SettingStore.forceDirectory(local);
        if (!System.getProperty("os.name").toLowerCase(java.util.Locale.ROOT).contains("win")) {
            assertTrue(flushed, "a Linux folder can be flushed");
        }
        assertFalse(SettingStore.forceDirectory(local.resolve("missing")));
    }

    @Test
    void noLocalFolderNoSave() {
        store.configure(new SettingStore.Locations(deploy, "FM2026", null));
        cams.number("a", 0, "");
        assertEquals(null, store.prepareSave("now"));
    }

    // ── Robot folder and hardware stamps ────────────────────────────────────

    private static final double[] THIS_ROBOT = {0.1, 0.2, 0.3, 0.4};
    private static final double[] SAVING = {0.15, 0.2, 0.3, 0.4};
    private static final double[] OTHER_ROBOT = {-0.3, 0.05, 0.45, -0.1};

    /** A chassis whose devices read {@code atBoot}, and which saves {@link #SAVING} next. */
    private static HardwareStamp chassis(double[] atBoot) {
        return new HardwareStamp(
                "Chassis", "test offsets", atBoot, 0.001, () -> List.of(THIS_ROBOT, SAVING));
    }

    private void configureStamped(double[] atBoot) {
        store.configure(
                new SettingStore.Locations(
                        deploy, "FM2026", local, Map.of("cams", chassis(atBoot))));
    }

    private static String stampJson(double[]... accept) {
        StringBuilder b = new StringBuilder("{\"what\": \"test offsets\", \"accept\": [");
        for (int i = 0; i < accept.length; i++) {
            b.append(i == 0 ? "" : ", ").append(java.util.Arrays.toString(accept[i]));
        }
        return b.append("]}").toString();
    }

    @Test
    void refusedInFileNamesAFileSavedOnAnotherRobot() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(localFile(), "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"a\": 3}");
        configureStamped(THIS_ROBOT);
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(1.0, a.get());
        assertTrue(store.refusedInFile(a));
    }

    @Test
    void anotherRobotConfigsLocalFolderIsIgnored() throws IOException {
        file(local.resolve("PM2026").resolve("cams.json"), "{\"a\": 9}");
        configure();
        assertEquals(0.0, cams.number("a", 0, "").get());
    }

    @Test
    void stampedFileFromThisRobotIsUsed() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(localFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(new double[] {0.1004, 0.2, 0.3, 0.4});
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(3.0, a.get());
        assertEquals(SettingSource.LOCAL, a.source());
        assertFalse(
                hasProblem(SettingStore.Level.INFO, "no code declares"), "the stamp is no setting");
        assertTrue(store.problems().keySet().stream().noneMatch(k -> k.startsWith("file:")));
    }

    @Test
    void stampedFileFromAnotherRobotIsRefusedAndGitIsUsed() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(localFile(), "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"a\": 3}");
        configureStamped(THIS_ROBOT);
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(1.0, a.get());
        assertEquals(SettingSource.ROBOT, a.source());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "was saved on another robot"));
        assertTrue(hasProblem(SettingStore.Level.ERROR, "its cams values are not used"));
    }

    @Test
    void unstampedFileOfAHardStopGroupIsRefused() throws IOException {
        file(localFile(), "{\"a\": 3}");
        configureStamped(THIS_ROBOT);
        assertEquals(0.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "has no hardware stamp"));
    }

    @Test
    void unreadableHardwareRefusesTheFile() throws IOException {
        file(localFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(null);
        assertEquals(0.0, cams.number("a", 0, "").get());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "could not be read at boot"));
    }

    @Test
    void groupWithoutAStampIgnoresOne() throws IOException {
        file(
                robotLocal().resolve("other.json"),
                "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"b\": 2}");
        configureStamped(THIS_ROBOT);
        DoubleSetting b = new SettingGroup("other", store::register).number("b", 0, "");
        assertEquals(2.0, b.get());
        assertEquals(SettingSource.LOCAL, b.source());
        assertFalse(hasProblem(SettingStore.Level.INFO, "no code declares"));
    }

    /** Save on this robot, boot it again, then boot another robot with the same controller. */
    @Test
    void savedStampMatchesThisRobotAfterTheRebootAndNotAnother() throws IOException {
        configureStamped(THIS_ROBOT);
        var t = new FakeTransport();
        store.enableLive(t);
        cams.number("a", 0, "");
        t.values.put("cams/a", 2.0);
        store.applyLive(true);
        SettingStore.write(store.prepareSave("now"));
        ObjectNode saved = SettingStore.parseObject(localFile());
        assertEquals(HardwareStamp.KEY, saved.fieldNames().next(), "stamp first");
        assertEquals(2, saved.get(HardwareStamp.KEY).get("accept").size());

        for (double[] reading : List.of(THIS_ROBOT, SAVING)) {
            var next = new SettingStore();
            next.configure(
                    new SettingStore.Locations(
                            deploy, "FM2026", local, Map.of("cams", chassis(reading))));
            DoubleSetting a = new SettingGroup("cams", next::register).number("a", 0, "");
            assertEquals(2.0, a.get());
            assertEquals(SettingSource.LOCAL, a.source());
        }

        var other = new SettingStore();
        other.configure(
                new SettingStore.Locations(
                        deploy, "FM2026", local, Map.of("cams", chassis(OTHER_ROBOT))));
        DoubleSetting a = new SettingGroup("cams", other::register).number("a", 0, "");
        assertEquals(0.0, a.get());
        assertTrue(
                other.problems().values().stream()
                        .anyMatch(p -> p.level() == SettingStore.Level.ERROR));
    }

    @Test
    void saveMovesARefusedFileAsideAndDropsItsValues() throws IOException {
        file(localFile(), "{\"$hardware\": " + stampJson(OTHER_ROBOT) + ", \"a\": 3, \"x\": 1}");
        configureStamped(THIS_ROBOT);
        cams.number("a", 0, "");
        SettingStore.write(store.prepareSave("now"));
        assertTrue(
                Files.readString(robotLocal().resolve("cams.json.refused")).contains("\"a\": 3"));
        ObjectNode saved = SettingStore.parseObject(localFile());
        assertFalse(saved.has("a"));
        assertFalse(saved.has("x"), "the other robot's undeclared keys are not carried over");
        assertTrue(saved.has(HardwareStamp.KEY));
    }

    // ── Files saved before the folder was per robot ─────────────────────────

    private Path legacyFile() {
        return local.resolve("cams.json");
    }

    @Test
    void oldFolderFileIsReadUntilTheNextSaveMovesIt() throws IOException {
        file(legacyFile(), "{\"a\": 3}");
        configure();
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(3.0, a.get());
        assertEquals(SettingSource.LOCAL, a.source());
        assertTrue(hasProblem(SettingStore.Level.INFO, "the next save moves it to"));

        SettingStore.write(store.prepareSave("now"));
        assertFalse(Files.exists(legacyFile()));
        assertTrue(Files.exists(local.resolve("cams.json.moved")));
        assertEquals(3, SettingStore.parseObject(localFile()).get("a").intValue());

        var next = new SettingStore();
        next.configure(new SettingStore.Locations(deploy, "FM2026", local));
        DoubleSetting a2 = new SettingGroup("cams", next::register).number("a", 0, "");
        assertEquals(3.0, a2.get());
        assertTrue(next.problems().keySet().stream().noneMatch(k -> k.startsWith("file:")));
    }

    @Test
    void oldFolderFileIsIgnoredOnceThisRobotsExists() throws IOException {
        file(legacyFile(), "{\"a\": 9}");
        file(localFile(), "{\"a\": 3}");
        configure();
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertFalse(hasProblem(SettingStore.Level.INFO, "the next save moves it"));
    }

    @Test
    void oldFolderFileOfAHardStopGroupIsRefusedAndRetiredBySave() throws IOException {
        file(robotFile("FM2026"), "{\"a\": 1}");
        file(legacyFile(), "{\"a\": 3}");
        configureStamped(THIS_ROBOT);
        DoubleSetting a = cams.number("a", 0, "");
        assertEquals(1.0, a.get(), "an old file has no stamp, so it may be another robot's");
        assertEquals(SettingSource.ROBOT, a.source());
        assertTrue(hasProblem(SettingStore.Level.ERROR, "saved before files were per robot"));
        assertTrue(hasProblem(SettingStore.Level.ERROR, "has no hardware stamp"));

        SettingStore.write(store.prepareSave("now"));
        assertFalse(Files.exists(legacyFile()));
        assertEquals("{\"a\": 3}", Files.readString(local.resolve("cams.json.refused")));
        ObjectNode saved = SettingStore.parseObject(localFile());
        assertFalse(saved.has("a"));
        assertTrue(saved.has(HardwareStamp.KEY));
    }

    @Test
    void stampedOldFolderFileFromThisRobotIsMigrated() throws IOException {
        file(legacyFile(), "{\"$hardware\": " + stampJson(THIS_ROBOT) + ", \"a\": 3}");
        configureStamped(THIS_ROBOT);
        assertEquals(3.0, cams.number("a", 0, "").get());
    }

    @Test
    void noRobotNameKeepsTheFlatFolder() throws IOException {
        file(legacyFile(), "{\"a\": 3}");
        store.configure(new SettingStore.Locations(deploy, null, local));
        assertEquals(3.0, cams.number("a", 0, "").get());
        assertFalse(hasProblem(SettingStore.Level.INFO, "the next save moves it"));
    }
}
