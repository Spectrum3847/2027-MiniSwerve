package frc.spectrumLib.vision;

/** Where camera frames come from: the coprocessor over NetworkTables, or nothing (replay). */
public interface VisionIO {
    /** Main thread, each loop: every frame each camera sent since the last call. */
    void updateInputs(VisionInputs inputs);

    /** Replay: inputs come from the log. */
    VisionIO NONE = inputs -> {};
}
