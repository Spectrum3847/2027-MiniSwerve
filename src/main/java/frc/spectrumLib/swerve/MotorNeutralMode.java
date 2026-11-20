package frc.spectrumLib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.spectrumLib.hardware.PhoenixBackground;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;

/**
 * Brake or coast on one set of motors (the drives, or the steers), applied on {@link
 * PhoenixBackground} (a config apply blocks).
 *
 * <p>A newer request skips older applies still queued, so brake at enable never waits behind a
 * retrying coast. Each switch re-applies the motor's boot MotorOutput group with only NeutralMode
 * changed, so the inversion (a hard stop) is never rebuilt here. A full config apply (boot, or a
 * retry after a reboot) uses the neutral mode wanted at that moment.
 */
final class MotorNeutralMode {
    private final TalonFX[] motors;
    private final String name;
    private final MotorOutputConfigs[] bootOutput;
    private final AtomicIntegerArray applied; // 1 brake, 0 coast
    private final AtomicInteger generation = new AtomicInteger();
    private volatile boolean wanted = true;

    /**
     * @param motors filled in by the caller before the first apply
     * @param name background call name, for logs (e.g. "Swerve/DriveBrake"); "/i" is appended
     */
    MotorNeutralMode(TalonFX[] motors, String name) {
        this.motors = motors;
        this.name = name;
        bootOutput = new MotorOutputConfigs[motors.length];
        applied = new AtomicIntegerArray(motors.length);
    }

    /** The neutral mode a full config apply should use now. */
    boolean wantedBrake() {
        return wanted;
    }

    /**
     * Applies a motor's full config (blocking: boot or the background thread) and records its
     * MotorOutput group and the neutral mode it applied.
     */
    StatusCode applyFull(int i, TalonFXConfiguration config, boolean brake) {
        bootOutput[i] = config.MotorOutput;
        StatusCode code = motors[i].getConfigurator().apply(config);
        if (code.isOK()) {
            applied.set(i, brake ? 1 : 0);
        }
        return code;
    }

    /** Main thread: switch every motor to brake (true) or coast. Non-blocking. */
    void request(boolean brake) {
        wanted = brake;
        int gen = generation.incrementAndGet();
        for (int i = 0; i < motors.length; i++) {
            final int m = i;
            PhoenixBackground.run(
                    name + "/" + i,
                    () -> {
                        if (generation.get() != gen || bootOutput[m] == null) {
                            return StatusCode.OK; // superseded, or never configured yet
                        }
                        var out = bootOutput[m].clone();
                        out.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                        StatusCode code = motors[m].getConfigurator().apply(out);
                        if (code.isOK()) {
                            applied.set(m, brake ? 1 : 0);
                        }
                        return code;
                    },
                    null);
        }
    }

    /** Every motor is in brake, as last applied. */
    boolean allBrake() {
        for (int i = 0; i < motors.length; i++) {
            if (applied.get(i) != 1) {
                return false;
            }
        }
        return true;
    }
}
