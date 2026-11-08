package frc.spectrumLib.battery;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * The team's battery list: {@code UID,name} per line, {@code #} comments (the same {@code
 * batteries.csv} SpectrumBatteryLogger keeps). It names a battery whose tag has no name written on
 * it. Read once at boot; the robot never writes it.
 */
public final class BatteryNames {
    private final Map<String, String> names = new HashMap<>();

    /** An empty list. */
    public BatteryNames() {}

    /** Parses list text (a bad line is skipped). */
    public static BatteryNames parse(String text) {
        BatteryNames list = new BatteryNames();
        for (String line : text.split("\\R")) {
            String s = line.strip();
            if (s.isEmpty() || s.startsWith("#")) {
                continue;
            }
            String[] parts = s.split(",", 2);
            if (parts.length == 2 && !parts[0].isBlank()) {
                list.names.put(key(parts[0]), parts[1].strip());
            }
        }
        return list;
    }

    /**
     * Reads the list file, or returns an empty list if it is missing or unreadable (the tag's own
     * name and UID still identify the battery).
     */
    public static BatteryNames load(Path file) {
        try {
            return Files.exists(file)
                    ? parse(Files.readString(file, StandardCharsets.UTF_8))
                    : new BatteryNames();
        } catch (IOException e) {
            return new BatteryNames();
        }
    }

    /** Listed name for a UID, or "" if it is not listed. */
    public String name(String uid) {
        return names.getOrDefault(key(uid), "");
    }

    /** How many batteries are listed. */
    public int size() {
        return names.size();
    }

    private static String key(String uid) {
        return uid.strip().toUpperCase(Locale.ROOT);
    }
}
