package frc.spectrumSim.field;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * The floor's height across the field, so the sim robot rides over bumps and ramps. Built from a
 * field file ({@code fields/<name>.json} on the classpath, or a path), so a new game's field is a
 * new file, not new code. Each bump is a box footprint (center, length along its heading, width
 * across) whose top rises from both ends at its ramp angle and is capped at its height: a 2026
 * BUMP, a flat platform with ramps, or a ridge.
 *
 * <p>Field frame (m). Steps too steep to drive up belong in {@link Arena} as walls, not here.
 * Allocation-free lookups; immutable after loading, so safe from any thread.
 */
public final class Terrain {
    /** One bump: center, unit heading, half sizes, height, rise per meter of ramp. */
    private final double[] cx, cy, cos, sin, halfLength, halfWidth, height, slope;

    /** A flat floor. */
    public static final Terrain FLAT = new Terrain(new double[0][]);

    /**
     * @param bumps each {x, y, length, width, height, rampDeg, headingDeg}
     */
    public Terrain(double[][] bumps) {
        int n = bumps.length;
        cx = new double[n];
        cy = new double[n];
        cos = new double[n];
        sin = new double[n];
        halfLength = new double[n];
        halfWidth = new double[n];
        height = new double[n];
        slope = new double[n];
        for (int i = 0; i < n; i++) {
            double[] b = bumps[i];
            cx[i] = b[0];
            cy[i] = b[1];
            halfLength[i] = b[2] / 2;
            halfWidth[i] = b[3] / 2;
            height[i] = b[4];
            slope[i] = Math.tan(Math.toRadians(b[5]));
            cos[i] = Math.cos(Math.toRadians(b[6]));
            sin[i] = Math.sin(Math.toRadians(b[6]));
        }
    }

    /** Floor height at a field point (m). Overlapping bumps take the higher. */
    public double height(double x, double y) {
        double z = 0;
        for (int i = 0; i < cx.length; i++) {
            double dx = x - cx[i], dy = y - cy[i];
            double along = Math.abs(dx * cos[i] + dy * sin[i]);
            double across = Math.abs(-dx * sin[i] + dy * cos[i]);
            if (along < halfLength[i] && across < halfWidth[i]) {
                z = Math.max(z, Math.min(height[i], (halfLength[i] - along) * slope[i]));
            }
        }
        return z;
    }

    /** Bumps in this terrain. */
    public int bumps() {
        return cx.length;
    }

    /**
     * Loads a field: {@code flat}, a name under {@code fields/} on the classpath (e.g. {@code
     * 2026-rebuilt}), or a path to a JSON file.
     */
    public static Terrain load(String name) {
        if (name == null || name.isBlank() || "flat".equals(name)) {
            return FLAT;
        }
        try (InputStream in = open(name)) {
            JsonNode bumps = new ObjectMapper().readTree(in).path("bumps");
            double[][] b = new double[bumps.size()][];
            for (int i = 0; i < b.length; i++) {
                JsonNode j = bumps.get(i);
                b[i] =
                        new double[] {
                            req(j, "x"),
                            req(j, "y"),
                            req(j, "length"),
                            req(j, "width"),
                            req(j, "height"),
                            req(j, "rampDeg"),
                            j.path("headingDeg").asDouble(0)
                        };
            }
            return new Terrain(b);
        } catch (IOException e) {
            throw new IllegalArgumentException("Cannot read terrain " + name + ": " + e, e);
        }
    }

    private static InputStream open(String name) throws IOException {
        InputStream in = Terrain.class.getResourceAsStream("/fields/" + name + ".json");
        if (in != null) {
            return in;
        }
        Path p = Path.of(name);
        if (Files.isRegularFile(p)) {
            return Files.newInputStream(p);
        }
        throw new IOException("no fields/" + name + ".json and no such file");
    }

    private static double req(JsonNode j, String key) {
        if (!j.path(key).isNumber()) {
            throw new IllegalArgumentException("bump needs a number " + key + ": " + j);
        }
        return j.path(key).asDouble();
    }
}
