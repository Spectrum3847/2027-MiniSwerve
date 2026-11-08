package frc.spectrumLib.battery;

import frc.spectrumLib.telemetry.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * Warns about a low battery before a match, judged only at rest: disabled for a while, so the
 * reading is the battery's resting voltage and not normal sag under load. Logs that resting voltage
 * so each battery's history (with {@code Battery/Name}) comes from the logs.
 *
 * <p>It never acts on the robot; it only alerts. Readings outside 6..16 V (no motor reporting) are
 * ignored.
 */
public final class BatteryVoltageCheck {
    private final double lowVolts;
    private final double settleSeconds;
    private final Alert low = new Alert("", Alert.Level.MEDIUM);

    private double disabledSince = Double.NaN;
    private double resting = Double.NaN;
    private double shown = Double.NaN;

    /**
     * @param lowVolts resting voltage below which to warn (V)
     * @param settleSeconds how long disabled before the reading counts as resting (s)
     */
    public BatteryVoltageCheck(double lowVolts, double settleSeconds) {
        this.lowVolts = lowVolts;
        this.settleSeconds = settleSeconds;
    }

    /**
     * Main loop, each loop.
     *
     * @param volts battery voltage from a logged input (V)
     * @param disabled whether the robot is disabled
     * @param now robot time (s)
     */
    public void periodic(double volts, boolean disabled, double now) {
        if (!disabled) {
            disabledSince = Double.NaN;
        } else if (Double.isNaN(disabledSince)) {
            disabledSince = now;
        }
        boolean settled = !Double.isNaN(disabledSince) && now - disabledSince >= settleSeconds;
        boolean valid = volts >= 6.0 && volts <= 16.0;
        if (settled && valid) {
            resting = volts;
        }
        boolean isLow = !Double.isNaN(resting) && resting < lowVolts;
        // Reword only when the reading moves, so the loop does not format a string every time.
        if (isLow && !(Math.abs(resting - shown) < 0.05)) {
            shown = resting;
            low.setText(String.format("Battery low at rest: %.2f V (swap it)", resting));
        }
        // Stays up through the match once seen: a low battery does not recover under load.
        low.set(isLow);

        // Battery voltage after settleSeconds disabled (V); the last resting reading while enabled.
        Logger.recordOutput("Battery/RestingVoltage", resting);
        // Resting voltage is below the low-battery threshold.
        Logger.recordOutput("Battery/Low", isLow);
    }

    /** Last resting voltage (V), NaN until the robot has rested once. */
    public double restingVolts() {
        return resting;
    }
}
