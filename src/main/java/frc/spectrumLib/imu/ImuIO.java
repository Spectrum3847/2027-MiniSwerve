package frc.spectrumLib.imu;

/** Where a second IMU's readings come from: the USB board, or nothing (sim, replay). */
public interface ImuIO {
    /** Main thread, each loop: copy the latest readings into the inputs. Never blocks. */
    void updateInputs(ImuInputs inputs);

    /** Sim and replay: no board (replay reads the inputs from the log). */
    ImuIO NONE = inputs -> {};
}
