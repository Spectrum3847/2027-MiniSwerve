package frc.spectrumSim.field;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * One season's field from a file: its size, the floor's bumps ({@link Terrain}) and the solid
 * elements bumpers hit (hubs, trench posts). A new game is a new file under {@code fields/} on the
 * classpath (see {@code src/main/resources/fields/README.md}), not new code.
 *
 * @param length field length (m), alliance wall to alliance wall
 * @param width field width (m), guardrail to guardrail
 * @param terrain the floor's bumps and ramps
 * @param elements solid elements on the field, as outlines (field frame)
 */
public record FieldMap(double length, double width, Terrain terrain, List<ConvexPolygon> elements) {
    private static final double INCH = 0.0254;

    /**
     * Loads a field: a name under {@code fields/} on the classpath (e.g. {@code 2026-rebuilt}), or
     * a path to a JSON file.
     */
    public static FieldMap load(String name) {
        try (InputStream in = open(name)) {
            JsonNode root = new ObjectMapper().readTree(in);
            String units = root.path("units").asText("m");
            double k =
                    switch (units) {
                        case "m" -> 1.0;
                        case "in" -> INCH;
                        default ->
                                throw new IllegalArgumentException("units are m or in: " + units);
                    };
            JsonNode bumps = root.path("bumps");
            double[][] b = new double[bumps.size()][];
            for (int i = 0; i < b.length; i++) {
                JsonNode j = bumps.get(i);
                b[i] =
                        new double[] {
                            k * req(j, "x"),
                            k * req(j, "y"),
                            k * req(j, "length"),
                            k * req(j, "width"),
                            k * req(j, "height"),
                            req(j, "rampDeg"),
                            j.path("headingDeg").asDouble(0),
                            k * j.path("edge").asDouble(0)
                        };
            }
            List<ConvexPolygon> elements = new ArrayList<>();
            for (JsonNode j : root.path("elements")) {
                elements.add(
                        ConvexPolygon.box(
                                k * req(j, "x"),
                                k * req(j, "y"),
                                k * req(j, "length"),
                                k * req(j, "width"),
                                Math.toRadians(j.path("headingDeg").asDouble(0))));
            }
            return new FieldMap(
                    k * req(root, "length"), k * req(root, "width"), new Terrain(b), elements);
        } catch (IOException e) {
            throw new IllegalArgumentException("Cannot read field " + name + ": " + e, e);
        }
    }

    private static InputStream open(String name) throws IOException {
        InputStream in = FieldMap.class.getResourceAsStream("/fields/" + name + ".json");
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
            throw new IllegalArgumentException("field file needs a number " + key + ": " + j);
        }
        return j.path(key).asDouble();
    }
}
