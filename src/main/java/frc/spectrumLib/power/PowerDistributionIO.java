package frc.spectrumLib.power;

/** Where power distribution readings come from: the board through WPILib, or nothing. */
public interface PowerDistributionIO {
    /** Main thread, each loop: copy the latest readings into the inputs. Never blocks. */
    void updateInputs(PowerDistributionInputs inputs);

    /** Sim and replay: no board (replay reads the inputs from the log). */
    PowerDistributionIO NONE = inputs -> {};
}
