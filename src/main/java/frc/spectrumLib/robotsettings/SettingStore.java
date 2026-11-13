package frc.spectrumLib.robotsettings;

import com.fasterxml.jackson.core.StreamReadFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.JsonMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * The settings registry: resolves each setting through the file layers, applies live changes, and
 * prepares saves. No WPILib or AdvantageKit here, so the precedence rules can be unit tested;
 * {@link RobotSettings} wires it to the logger, alerts and the transport.
 *
 * <p>Over 400 lines: ported whole from FM Params; to be split into layer reading, saving and
 * problems.
 *
 * <p>Index: Locations | register | layers/read | applyLive | prepareSave/write | problems
 */
final class SettingStore {
    /**
     * Strict about duplicate keys: with two entries for one key, which one wins is invisible in the
     * file, so the file is unreadable instead.
     */
    static final ObjectMapper JSON =
            JsonMapper.builder().enable(StreamReadFeature.STRICT_DUPLICATE_DETECTION).build();

    private static final String TEMP_SUFFIX = ".tmp";

    enum Level {
        ERROR,
        WARNING,
        INFO
    }

    record Problem(Level level, String text) {}

    /**
     * Where the files are. A null folder or robot skips that layer.
     *
     * @param deployDir the deploy folder's {@code settings} directory
     * @param robot this robot's name, for {@code robots/<robot>/} and the local folder
     * @param localRoot the robot-local folder that survives deploys; this robot's files are in
     *     {@code <localRoot>/<robot>/}
     * @param stamps the groups whose local file must have been saved on this robot, by group
     */
    record Locations(
            Path deployDir, String robot, Path localRoot, Map<String, HardwareStamp> stamps) {
        static final Locations NONE = new Locations(null, null, null);

        Locations(Path deployDir, String robot, Path localRoot) {
            this(deployDir, robot, localRoot, Map.of());
        }

        /** This robot's local folder, or null for none. */
        Path localDir() {
            if (localRoot == null) {
                return null;
            }
            return robot == null || robot.isEmpty() ? localRoot : localRoot.resolve(robot);
        }

        /**
         * Where files were saved before they were per robot: {@code <localRoot>/<group>.json}. Null
         * when that is this robot's folder anyway.
         */
        Path legacyDir() {
            Path dir = localDir();
            return dir == null || dir.equals(localRoot) ? null : localRoot;
        }
    }

    /**
     * One file as read at boot. {@code values} is null when it is missing, unreadable or refused (a
     * hard-stop group's file not saved on this robot).
     */
    record Layer(SettingSource source, Path path, String label, ObjectNode values, String status) {
        boolean unreadable() {
            return status.startsWith("unreadable");
        }

        boolean refused() {
            return status.startsWith("refused");
        }
    }

    /**
     * What a save writes: file name to contents, the files to move aside first, the old-folder
     * files to rename once the new ones are written, and each saved value by path.
     */
    record SaveJob(
            Path dir,
            Map<String, ObjectNode> files,
            Map<Path, Path> moveAside,
            Map<Path, Path> retire,
            Map<String, Object> values) {}

    private Locations locations = Locations.NONE;
    private final Map<String, Setting<?>> settings = new LinkedHashMap<>();
    private final Map<String, List<Layer>> layers = new LinkedHashMap<>();
    private final Map<String, Problem> problems = new LinkedHashMap<>();
    private int problemsVersion;
    private int valuesVersion;

    private SettingTransport transport;
    private final Map<Setting<?>, SettingTransport.LiveValue> live = new LinkedHashMap<>();
    private final Set<String> pendingLive = new LinkedHashSet<>();
    private final Map<String, Object> savedValues = new HashMap<>();

    /**
     * Local files a save has written this boot. The local layer is read once at boot, so without
     * this a second save could not tell that the first one made a file: tuning a value back to its
     * base and saving again wrote nothing, and the next boot ran the first save's value.
     */
    private final java.util.Set<String> writtenThisBoot = new java.util.HashSet<>();

    // ── Declaring and resolving ─────────────────────────────────────────────

    /** Points the store at its files and re-resolves every setting already declared. */
    void configure(Locations loc) {
        locations = loc;
        if (loc.localDir() != null) {
            deleteStaleTemps(loc.localDir());
        }
        if (loc.legacyDir() != null) {
            deleteStaleTemps(loc.legacyDir());
        }
        layers.clear();
        problems.keySet()
                .removeIf(
                        k ->
                                k.startsWith("file:")
                                        || k.startsWith("value:")
                                        || k.startsWith("unknown:"));
        problemsVersion++;
        for (Setting<?> p : settings.values()) {
            resolve(p);
        }
        refreshSummaries();
    }

    void register(Setting<?> p) {
        if (settings.containsKey(p.path())) {
            throw new IllegalArgumentException("Setting " + p.path() + " is declared twice");
        }
        settings.put(p.path(), p);
        resolve(p);
        if (transport != null) {
            live.put(p, transport.publish(p));
        }
        refreshSummaries();
    }

    Collection<Setting<?>> settings() {
        return Collections.unmodifiableCollection(settings.values());
    }

    Setting<?> find(String path) {
        return settings.get(path);
    }

    private <T> void resolve(Setting<T> p) {
        T value = p.getDefault();
        SettingSource source = SettingSource.DEFAULT;
        T base = value;
        for (Layer layer : layers(p.group())) {
            if (layer.values() == null) {
                continue;
            }
            JsonNode node = layer.values().get(p.key());
            String id = "value:" + layer.path() + ":" + p.key();
            if (node == null) {
                clearProblem(id);
                continue;
            }
            try {
                value = p.fromJson(unwrap(node));
                source = layer.source();
                clearProblem(id);
            } catch (IllegalArgumentException e) {
                problem(
                        id,
                        Level.WARNING,
                        "RobotSettings: "
                                + layer.label()
                                + ": "
                                + p.key()
                                + " "
                                + e.getMessage()
                                + "; using "
                                + value
                                + " ("
                                + source.name().toLowerCase(Locale.ROOT)
                                + ")");
            }
            if (layer.source() != SettingSource.LOCAL) {
                base = value;
            }
        }
        p.setBase(base);
        p.set(value, source);
        valuesVersion++;
    }

    /** A file value is either the value itself or an object holding it under {@code value}. */
    static JsonNode unwrap(JsonNode node) {
        return node.isObject() && node.has("value") ? node.get("value") : node;
    }

    private List<Layer> layers(String group) {
        List<Layer> cached = layers.get(group);
        if (cached != null) {
            return cached;
        }
        List<Layer> out = new ArrayList<>(4);
        String file = group + ".json";
        if (locations.deployDir() != null) {
            out.add(
                    read(
                            SettingSource.DEPLOY,
                            locations.deployDir().resolve(file),
                            "settings/" + file));
            if (locations.robot() != null && !locations.robot().isEmpty()) {
                out.add(
                        read(
                                SettingSource.ROBOT,
                                locations
                                        .deployDir()
                                        .resolve("robots")
                                        .resolve(locations.robot())
                                        .resolve(file),
                                "settings/robots/" + locations.robot() + "/" + file));
            }
        }
        if (locations.localDir() != null) {
            out.add(readLocal(group, file));
        }
        layers.put(group, out);
        return out;
    }

    /**
     * This robot's local file for a group. Until the first save after an update, the file may still
     * be in the folder files were saved to before they were per robot; it is read from there, and
     * the save moves it.
     */
    private Layer readLocal(String group, String file) {
        Path path = locations.localDir().resolve(file);
        Path legacyDir = locations.legacyDir();
        Path legacy = legacyDir == null ? null : legacyDir.resolve(file);
        if (Files.exists(path) || legacy == null || !Files.exists(legacy)) {
            return checkStamp(group, read(SettingSource.LOCAL, path, path.toString()));
        }
        Layer layer =
                checkStamp(
                        group,
                        read(
                                SettingSource.LOCAL,
                                legacy,
                                legacy + " (saved before files were per robot)"));
        if (layer.values() != null) {
            problem(
                    "file:legacy:" + legacy,
                    Level.INFO,
                    "RobotSettings: "
                            + group
                            + " comes from "
                            + legacy
                            + "; the next save moves it to "
                            + locations.localDir());
        }
        return layer;
    }

    /**
     * Takes the stamp out of a local file's values and, for a hard-stop group, refuses a file not
     * saved on this robot.
     */
    private Layer checkStamp(String group, Layer layer) {
        if (layer.values() == null) {
            return layer;
        }
        JsonNode stamp = layer.values().remove(HardwareStamp.KEY);
        HardwareStamp hardware = locations.stamps().get(group);
        String why = hardware == null ? null : hardware.mismatch(stamp);
        if (why == null) {
            return layer;
        }
        problem(
                "file:" + layer.path(),
                Level.ERROR,
                "RobotSettings: "
                        + layer.label()
                        + " "
                        + why
                        + "; its "
                        + group
                        + " values are not used");
        return new Layer(layer.source(), layer.path(), layer.label(), null, "refused: " + why);
    }

    private Layer read(SettingSource source, Path path, String label) {
        String id = "file:" + path;
        if (!Files.exists(path)) {
            clearProblem(id);
            return new Layer(source, path, label, null, "missing");
        }
        try {
            ObjectNode values = parseObject(path);
            clearProblem(id);
            return new Layer(source, path, label, values, "loaded");
        } catch (IOException | RuntimeException e) {
            String why = shortMessage(e);
            problem(
                    id,
                    Level.ERROR,
                    "RobotSettings: "
                            + label
                            + " is unreadable ("
                            + why
                            + "); its values are not used");
            return new Layer(source, path, label, null, "unreadable: " + why);
        }
    }

    static ObjectNode parseObject(Path path) throws IOException {
        JsonNode root = JSON.readTree(Files.readString(path, StandardCharsets.UTF_8));
        if (root == null || !root.isObject()) {
            throw new IOException("not a JSON object");
        }
        return (ObjectNode) root;
    }

    private static String shortMessage(Exception e) {
        String m = e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage();
        m = m.lines().findFirst().orElse(m);
        return m.length() > 120 ? m.substring(0, 120) + "..." : m;
    }

    /** Every layer read so far, for the log. */
    List<Layer> loadedLayers() {
        List<Layer> out = new ArrayList<>();
        layers.values().forEach(out::addAll);
        return out;
    }

    /**
     * Whether a file layer already read gave this setting a value it refused (wrong type or out of
     * bounds), could not be read at all where it exists, or was refused whole (a hard-stop file
     * saved on another robot). Reads nothing from disk and changes nothing.
     */
    boolean refusedInFile(Setting<?> p) {
        List<Layer> group = layers.get(p.group());
        if (group == null) {
            return false;
        }
        for (Layer layer : group) {
            if (layer.unreadable() || layer.refused()) {
                return true;
            }
            JsonNode node = layer.values() == null ? null : layer.values().get(p.key());
            if (node == null) {
                continue;
            }
            try {
                p.fromJson(unwrap(node));
            } catch (IllegalArgumentException e) {
                return true;
            }
        }
        return false;
    }

    // ── Checking a file without applying it ─────────────────────────────────

    /**
     * Problems with one settings file against the declared settings: unreadable, a key nobody
     * declares, or a value of the wrong type or out of bounds. The file's name is its group.
     */
    List<String> checkFile(Path path) {
        List<String> out = new ArrayList<>();
        String name = path.getFileName().toString();
        String group = name.endsWith(".json") ? name.substring(0, name.length() - 5) : name;
        ObjectNode values;
        try {
            values = parseObject(path);
        } catch (IOException | RuntimeException e) {
            out.add(path + ": unreadable (" + shortMessage(e) + ")");
            return out;
        }
        for (var e : values.properties()) {
            Setting<?> p = settings.get(group + "/" + e.getKey());
            if (p == null) {
                out.add(path + ": " + e.getKey() + " is not declared in group " + group);
                continue;
            }
            try {
                p.fromJson(unwrap(e.getValue()));
            } catch (IllegalArgumentException ex) {
                out.add(path + ": " + e.getKey() + " " + ex.getMessage());
            }
        }
        return out;
    }

    // ── Live changes ────────────────────────────────────────────────────────

    /**
     * Publishes every setting on the transport; settings declared later are published as they come.
     */
    void enableLive(SettingTransport t) {
        transport = t;
        for (Setting<?> p : settings.values()) {
            live.computeIfAbsent(p, t::publish);
        }
    }

    boolean liveEnabled() {
        return transport != null;
    }

    /**
     * Applies the transport's requested values, only when {@code allowed} ({@link
     * RobotSettings#liveChangesAllowed}). A request made while not allowed waits, and applies once
     * allowed unless it is changed back first.
     */
    void applyLive(boolean allowed) {
        if (transport == null) {
            return;
        }
        boolean changed = false;
        for (var e : live.entrySet()) {
            changed |= applyLive(e.getKey(), e.getValue(), allowed);
        }
        if (changed) {
            refreshSummaries();
        }
    }

    private <T> boolean applyLive(Setting<T> p, SettingTransport.LiveValue h, boolean allowed) {
        Object requested = h.get();
        if (requested == null) {
            return false;
        }
        String id = "live:" + p.path();
        T v;
        try {
            v = p.fromLive(requested);
        } catch (IllegalArgumentException e) {
            problem(
                    id,
                    Level.WARNING,
                    "Tuning: " + p.path() + " refused, " + e.getMessage() + "; kept " + p.get());
            h.set(p.get());
            pendingLive.remove(p.path());
            return false;
        }
        if (v.equals(p.get())) {
            pendingLive.remove(p.path());
            return false;
        }
        if (!allowed) {
            pendingLive.add(p.path());
            return false;
        }
        pendingLive.remove(p.path());
        clearProblem(id);
        p.set(v, SettingSource.LIVE);
        valuesVersion++;
        return true;
    }

    /** RobotSettings whose dashboard value differs from the applied one, waiting until allowed. */
    Set<String> pendingLive() {
        return Collections.unmodifiableSet(pendingLive);
    }

    /** Sets a value restored from the log (replay). */
    void restore(Setting<?> p, Object value, SettingSource source) {
        if (!value.equals(p.get()) || source != p.source()) {
            p.setUnchecked(value, source);
            valuesVersion++;
            refreshSummaries();
        }
    }

    // ── Saving ──────────────────────────────────────────────────────────────

    /**
     * What a save would write to the robot-local folder: every value that differs from the layers
     * below it, with the value it replaced and when. A value loaded from the local file keeps its
     * entry as it was; keys nobody declares are kept. Null when there is no local folder.
     */
    SaveJob prepareSave(String savedAt) {
        if (locations.localDir() == null) {
            return null;
        }
        Map<String, List<Setting<?>>> byGroup = new LinkedHashMap<>();
        for (Setting<?> p : settings.values()) {
            byGroup.computeIfAbsent(p.group(), g -> new ArrayList<>()).add(p);
        }
        Map<String, ObjectNode> files = new TreeMap<>();
        Map<Path, Path> moveAside = new LinkedHashMap<>();
        Map<Path, Path> retire = new LinkedHashMap<>();
        Map<String, Object> saved = new LinkedHashMap<>();
        for (var e : byGroup.entrySet()) {
            String file = e.getKey() + ".json";
            Layer local = localLayer(e.getKey());
            ObjectNode existing = local == null ? null : local.values();
            ObjectNode out = JSON.createObjectNode();
            if (existing != null) {
                for (var kv : existing.properties()) {
                    if (!settings.containsKey(e.getKey() + "/" + kv.getKey())) {
                        out.set(kv.getKey(), kv.getValue().deepCopy());
                    }
                }
            }
            for (Setting<?> p : e.getValue()) {
                if (p.get().equals(p.base())) {
                    continue;
                }
                saved.put(p.path(), p.get());
                JsonNode kept = existing == null ? null : existing.get(p.key());
                if (p.source() == SettingSource.LOCAL && kept != null) {
                    out.set(p.key(), kept.deepCopy());
                } else {
                    ObjectNode entry = out.putObject(p.key());
                    entry.set("value", p.valueJson());
                    entry.set("replaced", p.baseJson());
                    entry.put("savedAt", savedAt);
                }
            }
            boolean hadFile = local != null && !"missing".equals(local.status());
            boolean rewritten = writtenThisBoot.contains(file);
            if (!out.isEmpty() || hadFile || rewritten) {
                files.put(file, stamped(e.getKey(), out));
            }
            if (!hadFile || rewritten) {
                // Nothing to set aside: no file at boot, or the boot file was already set aside
                // by this boot's first save and the one there now is ours.
                continue;
            }
            String suffix = local.unreadable() ? ".bad" : local.refused() ? ".refused" : null;
            Path target = locations.localDir().resolve(file);
            if (!local.path().equals(target)) {
                retire.put(local.path(), sibling(local.path(), suffix == null ? ".moved" : suffix));
            } else if (suffix != null) {
                moveAside.put(target, sibling(target, suffix));
            }
        }
        return new SaveJob(locations.localDir(), files, moveAside, retire, saved);
    }

    /** A hard-stop group's file gets this robot's stamp first, then its values. */
    private ObjectNode stamped(String group, ObjectNode values) {
        HardwareStamp hardware = locations.stamps().get(group);
        if (hardware == null) {
            return values;
        }
        ObjectNode out = JSON.createObjectNode();
        out.set(HardwareStamp.KEY, hardware.stamp());
        out.setAll(values);
        return out;
    }

    private static Path sibling(Path path, String suffix) {
        return path.resolveSibling(path.getFileName() + suffix);
    }

    /** Records a save that was written, so its live values no longer count as unsaved. */
    void markSaved(SaveJob job) {
        savedValues.clear();
        savedValues.putAll(job.values());
        writtenThisBoot.addAll(job.files().keySet());
        refreshSummaries();
    }

    private Layer localLayer(String group) {
        for (Layer l : layers(group)) {
            if (l.source() == SettingSource.LOCAL) {
                return l;
            }
        }
        return null;
    }

    /**
     * Writes a save. Blocking file I/O: never on the main thread. Each file is written to a
     * temporary file, flushed to the disk and moved over the old one, then the folder is flushed,
     * so a power cut leaves the old file or the new one, never half of one or an empty one. An
     * unreadable file is kept as {@code <name>.bad}, and a hard-stop file saved on another robot as
     * {@code <name>.refused}, rather than overwritten. A file still in the folder files were saved
     * to before they were per robot is renamed once its replacement is written ({@code .moved}, or
     * {@code .bad} / {@code .refused}), so it is read only until then.
     */
    static void write(SaveJob job) throws IOException {
        Files.createDirectories(job.dir());
        for (var e : job.moveAside().entrySet()) {
            if (Files.exists(e.getKey())) {
                Files.move(e.getKey(), e.getValue(), StandardCopyOption.REPLACE_EXISTING);
            }
        }
        for (var e : job.files().entrySet()) {
            Path target = job.dir().resolve(e.getKey());
            Path tmp = job.dir().resolve(e.getKey() + TEMP_SUFFIX);
            writeAndForce(
                    tmp,
                    JSON.writerWithDefaultPrettyPrinter().writeValueAsString(e.getValue()) + "\n");
            try {
                Files.move(
                        tmp,
                        target,
                        StandardCopyOption.REPLACE_EXISTING,
                        StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException ex) {
                Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING);
            }
        }
        for (var e : job.retire().entrySet()) {
            if (Files.exists(e.getKey())) {
                Files.move(e.getKey(), e.getValue(), StandardCopyOption.REPLACE_EXISTING);
            }
        }
        forceDirectory(job.dir());
        for (Path retired : job.retire().values()) {
            if (!retired.getParent().equals(job.dir())) {
                forceDirectory(retired.getParent());
            }
        }
    }

    /**
     * Writes the file and waits until it is on the disk. Without this the rename can reach the disk
     * before the data, and a power cut leaves an empty file under the real name.
     */
    private static void writeAndForce(Path path, String text) throws IOException {
        try (FileChannel ch =
                FileChannel.open(
                        path,
                        StandardOpenOption.CREATE,
                        StandardOpenOption.TRUNCATE_EXISTING,
                        StandardOpenOption.WRITE)) {
            ByteBuffer buf = ByteBuffer.wrap(text.getBytes(StandardCharsets.UTF_8));
            while (buf.hasRemaining()) {
                ch.write(buf);
            }
            ch.force(true);
        }
    }

    /**
     * Flushes the folder itself, so the renames survive a power cut. Best effort: Windows cannot
     * open a folder this way, and the simulator is the only thing that saves there.
     */
    static boolean forceDirectory(Path dir) {
        try (FileChannel ch = FileChannel.open(dir, StandardOpenOption.READ)) {
            ch.force(true);
            return true;
        } catch (IOException | UnsupportedOperationException e) {
            return false;
        }
    }

    /**
     * Deletes the temporary files of a save that a power cut or crash interrupted. The real files
     * are whole, old or new; a temporary file is never read.
     */
    static void deleteStaleTemps(Path dir) {
        if (!Files.isDirectory(dir)) {
            return;
        }
        try (DirectoryStream<Path> temps = Files.newDirectoryStream(dir, "*.json" + TEMP_SUFFIX)) {
            for (Path tmp : temps) {
                Files.deleteIfExists(tmp);
            }
        } catch (IOException | RuntimeException e) {
            // Left for the next boot; the save overwrites it anyway. RuntimeException too: a
            // read error while listing arrives as DirectoryIteratorException, and settings must
            // never stop the robot booting.
        }
    }

    // ── Problems ────────────────────────────────────────────────────────────

    /** Current problems by id, in the order they were raised. */
    Map<String, Problem> problems() {
        return Collections.unmodifiableMap(problems);
    }

    /** Goes up whenever the problem list changes. */
    int problemsVersion() {
        return problemsVersion;
    }

    /** Goes up whenever any value or source changes. */
    int valuesVersion() {
        return valuesVersion;
    }

    void problem(String id, Level level, String text) {
        Problem p = new Problem(level, text);
        if (!p.equals(problems.get(id))) {
            problems.remove(id);
            problems.put(id, p);
            problemsVersion++;
        }
    }

    void clearProblem(String id) {
        if (problems.remove(id) != null) {
            problemsVersion++;
        }
    }

    /** Unknown keys in loaded files, and how many values come from outside git. */
    private void refreshSummaries() {
        for (var e : layers.entrySet()) {
            for (Layer l : e.getValue()) {
                if (l.values() == null) {
                    continue;
                }
                List<String> unknown = new ArrayList<>();
                for (var kv : l.values().properties()) {
                    if (!settings.containsKey(e.getKey() + "/" + kv.getKey())) {
                        unknown.add(kv.getKey());
                    }
                }
                String id = "unknown:" + l.path();
                if (unknown.isEmpty()) {
                    clearProblem(id);
                } else {
                    problem(
                            id,
                            Level.INFO,
                            "RobotSettings: "
                                    + l.label()
                                    + " has keys no code declares: "
                                    + String.join(", ", unknown));
                }
            }
        }
        int local = 0;
        int unsaved = 0;
        for (Setting<?> p : settings.values()) {
            boolean saved = p.get().equals(savedValues.get(p.path()));
            if (p.source() == SettingSource.LOCAL || (p.source() == SettingSource.LIVE && saved)) {
                local++;
            } else if (p.source() == SettingSource.LIVE) {
                unsaved++;
            }
        }
        summary("summary:local", local, " from this robot's local overrides, not in git");
        summary("summary:live", unsaved, " tuned live and not saved");
    }

    private void summary(String id, int n, String what) {
        if (n == 0) {
            clearProblem(id);
        } else {
            problem(id, Level.INFO, "RobotSettings: " + n + (n == 1 ? " value" : " values") + what);
        }
    }
}
