package frc.spectrumSim.field;

/**
 * A convex outline on the field floor (field frame, meters, vertices counter-clockwise): a wall, a
 * field element's footprint, or a robot's bumpers. Robot outlines move; {@link #setBox} rewrites
 * the vertices in place so no step allocates.
 */
public final class ConvexPolygon {
    final double[] x;
    final double[] y;

    // Corner offsets for setBox, reused (one thread steps the sim).
    private final double[] cornerX = new double[4];
    private final double[] cornerY = new double[4];

    /**
     * @param xy x0, y0, x1, y1, ... counter-clockwise
     */
    public ConvexPolygon(double... xy) {
        if (xy.length < 6 || xy.length % 2 != 0) {
            throw new IllegalArgumentException("need at least 3 vertices");
        }
        x = new double[xy.length / 2];
        y = new double[xy.length / 2];
        for (int i = 0; i < x.length; i++) {
            x[i] = xy[2 * i];
            y[i] = xy[2 * i + 1];
        }
    }

    /**
     * A box of {@code length} by {@code width}, centered at (cx, cy), turned by {@code heading}.
     */
    public static ConvexPolygon box(
            double cx, double cy, double length, double width, double heading) {
        var p = new ConvexPolygon(0, 0, 1, 0, 1, 1, 0, 1);
        p.setBox(cx, cy, length, width, heading);
        return p;
    }

    /** Moves a four-vertex box in place (a robot's bumpers each step). */
    public void setBox(double cx, double cy, double length, double width, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double hl = length / 2, hw = width / 2;
        cornerX[0] = hl;
        cornerX[1] = -hl;
        cornerX[2] = -hl;
        cornerX[3] = hl;
        cornerY[0] = hw;
        cornerY[1] = hw;
        cornerY[2] = -hw;
        cornerY[3] = -hw;
        double[] lx = cornerX;
        double[] ly = cornerY;
        // Counter-clockwise: front-left, back-left, back-right, front-right.
        for (int i = 0; i < 4; i++) {
            x[i] = cx + lx[i] * c - ly[i] * s;
            y[i] = cy + lx[i] * s + ly[i] * c;
        }
    }

    public int size() {
        return x.length;
    }

    public double x(int i) {
        return x[i];
    }

    public double y(int i) {
        return y[i];
    }
}
