package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;

/**
 * Sticky faults on the steer motors and CANcoders, as one bit mask per device. A live fault read at
 * 10 Hz misses anything shorter than a sample; a sticky fault latches it. Cleared at boot, so a set
 * bit means "happened since boot" (if the boot clear failed, since before boot).
 *
 * <p>Bits are {@link #STEER_BITS} and {@link #ENCODER_BITS}: index i is bit i. They answer why a
 * steer motor stops trusting its CANcoder: the data went missing or bad, the CANcoder reset or
 * browned out, or the motor itself did.
 */
final class SwerveStickyFaults {
    /** Steer TalonFX sticky faults, bit 0 first. */
    static final String[] STEER_BITS = {
        "RemoteSensorDataInvalid",
        "RemoteSensorReset",
        "FusedSensorOutOfSync",
        "BootDuringEnable",
        "Undervoltage",
        "UnstableSupplyV",
        "BridgeBrownout",
    };

    /** CANcoder sticky faults, bit 0 first. */
    static final String[] ENCODER_BITS = {
        "BadMagnet", "BootDuringEnable", "Undervoltage", "Hardware",
    };

    private final TalonFX[] steer;
    private final CANcoder[] encoder;
    private final BaseStatusSignal[][] steerSignals;
    private final BaseStatusSignal[][] encoderSignals;

    SwerveStickyFaults(TalonFX[] steer, CANcoder[] encoder) {
        this.steer = steer;
        this.encoder = encoder;
        int n = steer.length;
        steerSignals = new BaseStatusSignal[n][];
        encoderSignals = new BaseStatusSignal[n][];
        for (int i = 0; i < n; i++) {
            TalonFX s = steer[i];
            steerSignals[i] =
                    new BaseStatusSignal[] {
                        s.getStickyFault_RemoteSensorDataInvalid(),
                        s.getStickyFault_RemoteSensorReset(),
                        s.getStickyFault_FusedSensorOutOfSync(),
                        s.getStickyFault_BootDuringEnable(),
                        s.getStickyFault_Undervoltage(),
                        s.getStickyFault_UnstableSupplyV(),
                        s.getStickyFault_BridgeBrownout(),
                    };
            CANcoder e = encoder[i];
            encoderSignals[i] =
                    new BaseStatusSignal[] {
                        e.getStickyFault_BadMagnet(),
                        e.getStickyFault_BootDuringEnable(),
                        e.getStickyFault_Undervoltage(),
                        e.getStickyFault_Hardware(),
                    };
        }
    }

    /** Every signal, to read and rate-set with the other slow signals. */
    List<BaseStatusSignal> signals() {
        List<BaseStatusSignal> all = new ArrayList<>();
        for (int i = 0; i < steer.length; i++) {
            all.addAll(List.of(steerSignals[i]));
            all.addAll(List.of(encoderSignals[i]));
        }
        return all;
    }

    /** Clears every sticky fault. Blocking: boot only. Returns the first failure, or OK. */
    StatusCode clear() {
        StatusCode code = StatusCode.OK;
        for (int i = 0; i < steer.length; i++) {
            StatusCode s = steer[i].clearStickyFaults();
            StatusCode e = encoder[i].clearStickyFaults();
            code = !code.isOK() ? code : !s.isOK() ? s : e;
        }
        return code;
    }

    /** Writes each device's mask from the last refresh (no allocation). */
    void read(long[] steerOut, long[] encoderOut) {
        for (int i = 0; i < steer.length; i++) {
            steerOut[i] = mask(steerSignals[i]);
            encoderOut[i] = mask(encoderSignals[i]);
        }
    }

    private static long mask(BaseStatusSignal[] bits) {
        long m = 0;
        for (int b = 0; b < bits.length; b++) {
            if (bits[b].getValueAsDouble() != 0) {
                m |= 1L << b;
            }
        }
        return m;
    }
}
