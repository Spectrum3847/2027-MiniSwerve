package frc.spectrumLib.power;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;

/**
 * A CTRE PDP or REV PDH read through WPILib's {@link PowerDistribution}. WPILib decodes the board's
 * own status frames, so a PDP works with no Phoenix 6 firmware. The getters read the latest frame
 * the HAL has; none of them waits on the bus.
 *
 * <p>The board is opened once, at boot (opening and the version read may wait on the bus, which the
 * main loop must not). Finish the match: if opening fails, or a read throws, the inputs say why and
 * read 0; nothing else stops.
 */
public final class PowerDistributionIOWpilib implements PowerDistributionIO {
    /** Below this the readings are not a battery (no frames: the HAL reports 0 V). */
    private static final double MIN_BATTERY_VOLTS = 4.0;

    private final PowerDistribution board;
    private final double[] currents;
    private final String firmware;
    private final String openError;

    /**
     * Opens the board (boot only: may wait on the bus).
     *
     * @param bus the CAN bus the board is on
     * @param module its CAN ID
     * @param type CTRE (PDP) or REV (PDH)
     */
    public PowerDistributionIOWpilib(CANPort bus, int module, ModuleType type) {
        PowerDistribution made = null;
        String error = "";
        String version = "";
        try {
            made = new PowerDistribution(bus, module, type);
            try {
                var v = made.getVersion();
                // A CTRE PDP answers 0.0.0 (WPILib does not read its version): unread, not 0.
                boolean known = v.firmwareMajor != 0 || v.firmwareMinor != 0 || v.firmwareFix != 0;
                version =
                        known ? v.firmwareMajor + "." + v.firmwareMinor + "." + v.firmwareFix : "";
            } catch (RuntimeException e) {
                version = ""; // readings still work without the version
            }
        } catch (RuntimeException e) {
            made = null;
            error = "could not open " + type + " " + module + " on " + bus + ": " + e;
        }
        board = made;
        currents = new double[made != null ? made.getNumChannels() : 0];
        firmware = version;
        openError = error;
    }

    @Override
    public void updateInputs(PowerDistributionInputs inputs) {
        inputs.firmware = firmware;
        if (board == null) {
            clear(inputs, openError);
            return;
        }
        try {
            double volts = board.getVoltage();
            double total = board.getTotalCurrent();
            double temp = board.getTemperature();
            for (int i = 0; i < currents.length; i++) {
                currents[i] = board.getCurrent(i);
            }
            inputs.voltage = volts;
            inputs.totalCurrent = total;
            inputs.temperature = temp;
            inputs.channelCurrents = currents;
            inputs.connected = volts >= MIN_BATTERY_VOLTS;
            inputs.error = inputs.connected ? "" : "no readings (battery reads under 4 V)";
        } catch (RuntimeException e) {
            clear(inputs, e.toString());
        }
    }

    /** No good readings this loop: 0, so the log never mixes old values with new. */
    private void clear(PowerDistributionInputs inputs, String why) {
        java.util.Arrays.fill(currents, 0);
        inputs.voltage = 0;
        inputs.totalCurrent = 0;
        inputs.temperature = 0;
        inputs.channelCurrents = currents;
        inputs.connected = false;
        inputs.error = why;
    }
}
