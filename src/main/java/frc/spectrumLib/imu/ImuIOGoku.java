package frc.spectrumLib.imu;

import io.github.frcgokuimu.GokuIMU;

/**
 * A Flywoo Goku F7 running GokuIMU firmware, on USB serial, through its vendordep. The library
 * finds the board by its USB product name "GokuIMU" (VID/PID on Windows), so it never opens the
 * CANable's serial port. It reads the board on its own thread, reconnects after an unplug, and
 * raises its own disconnect and fault alerts (from its own thread, every 250 ms). Opening it also
 * makes the JVM ignore SIGHUP. Every call here reads its latest frame and does not wait;
 * getInputs() allocates a small record each loop, which this library offers no way around.
 *
 * <p>Read alongside the Pigeon for now, not used for heading. Finish the match: if opening fails or
 * a read throws, the inputs say why and nothing else stops.
 */
public final class ImuIOGoku implements ImuIO {
    private final GokuIMU imu;
    private final String openError;

    /**
     * Opens the board (boot only).
     *
     * @param updateHz frames per second to ask the board for (1-1000)
     */
    public ImuIOGoku(double updateHz) {
        GokuIMU made = null;
        String error = "";
        try {
            made = new GokuIMU(); // the first board whose USB product name is GokuIMU
            made.setUpdateFrequency(updateHz);
        } catch (RuntimeException e) {
            made = null;
            error = "could not open the Goku IMU: " + e;
        }
        imu = made;
        openError = error;
    }

    @Override
    public void updateInputs(ImuInputs inputs) {
        if (imu == null) {
            clear(inputs, openError);
            return;
        }
        try {
            GokuIMU.Inputs in = imu.getInputs();
            inputs.connected = in.connected();
            inputs.calibrated = in.calibrated();
            inputs.moving = in.moving();
            inputs.yawDeg = in.yawDeg();
            inputs.pitchDeg = in.pitchDeg();
            inputs.rollDeg = in.rollDeg();
            inputs.yawRateDps = in.yawRateWorldDps();
            inputs.timestamp = in.timestamp();
            inputs.temperatureC = in.temperatureC();
            inputs.faultFlags = in.faultFlags();
            inputs.stickyFaultFlags = in.stickyFaultFlags();
            inputs.error = in.connected() ? "" : "not connected";
            if (!in.connected()) {
                inputs.deviceInfo = ""; // read again after a reconnect: it may be another board
            }
            if (inputs.deviceInfo.isEmpty() && in.connected()) {
                GokuIMU.DeviceInfo d = imu.getDeviceInfo();
                if (d != null) {
                    inputs.deviceInfo =
                            d.firmwareVersion() + " " + d.board() + " " + d.serialNumber();
                }
            }
        } catch (RuntimeException e) {
            clear(inputs, e.toString());
        }
    }

    /** No good readings this loop: 0, so the log never mixes old values with new. */
    private static void clear(ImuInputs inputs, String why) {
        inputs.connected = false;
        inputs.calibrated = false;
        inputs.moving = false;
        inputs.yawDeg = 0;
        inputs.pitchDeg = 0;
        inputs.rollDeg = 0;
        inputs.yawRateDps = 0;
        inputs.timestamp = 0;
        inputs.temperatureC = 0;
        inputs.faultFlags = 0;
        inputs.stickyFaultFlags = 0;
        inputs.error = why;
    }
}
