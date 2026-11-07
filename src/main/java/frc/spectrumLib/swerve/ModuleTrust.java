package frc.spectrumLib.swerve;

import org.littletonrobotics.junction.Logger;

/**
 * What each module can still do ({@link ModuleMode}) and which modules odometry can trust, from
 * logged inputs only (so replay agrees).
 *
 * <p>Driving never stops for a module fault (AGENTS.md "Finish the match"): a bad drive motor makes
 * its module {@link ModuleMode#STEER_ONLY}; a bad steer motor, or an angle never known since boot,
 * makes it {@link ModuleMode#COAST}; the rest keep driving. A CANcoder lost after boot does not
 * change the mode: the steer motor keeps the angle from its own rotor.
 *
 * <p>Odometry is stricter: it uses a module only when it is {@link ModuleMode#NORMAL}, its CANcoder
 * is fused and readable, and that sample's signals were fresh.
 *
 * <p>Logged under {@code <prefix>/}: {@code Mode}, {@code Trusted}, {@code AbsoluteAngleKnown}.
 */
public final class ModuleTrust {
    private final String prefix;
    private final int n;
    private final double[] weight;
    private final boolean[] trusted;
    private final boolean[] absoluteKnown;
    private final ModuleMode[] mode;
    private final String[] modeNames;
    private int version;
    private final boolean[] cmdDrive;
    private final boolean[] cmdSteer;
    private final boolean[] cmdDriveSlow;
    private final boolean[] cmdSteerSlow;
    private ModuleCommands commands;

    public ModuleTrust(String prefix, int modules) {
        this.prefix = prefix;
        n = modules;
        weight = new double[n];
        trusted = new boolean[n];
        absoluteKnown = new boolean[n];
        mode = new ModuleMode[n];
        modeNames = new String[n];
        java.util.Arrays.fill(mode, ModuleMode.NORMAL);
        commands = ModuleCommands.allNormal(n);
        cmdDrive = commands.drive().clone();
        cmdSteer = commands.steer().clone();
        cmdDriveSlow = new boolean[n];
        cmdSteerSlow = new boolean[n];
    }

    private static boolean set(boolean[] a, int i, boolean v) {
        boolean changed = a[i] != v;
        a[i] = v;
        return changed;
    }

    /** How the swerve thread should command each motor (new object only when it changes). */
    public ModuleCommands commands() {
        return commands;
    }

    /** Updates modes and trust from health. Call once per main loop, after inputs. */
    public void update(SwerveHealth h) {
        boolean changed = false;
        for (int i = 0; i < n; i++) {
            boolean encoder = h.encoderConnected[i] && h.encoderConfigured[i] && h.magnetOk[i];
            if (encoder && h.steerConnected[i] && h.steerConfigured[i] && h.fusedOk[i]) {
                absoluteKnown[i] = true;
            }
            boolean steerUsable = h.steerConnected[i] && h.steerConfigured[i] && absoluteKnown[i];
            boolean driveUsable = h.driveConnected[i] && h.driveConfigured[i];
            ModuleMode m =
                    !steerUsable
                            ? ModuleMode.COAST
                            : driveUsable ? ModuleMode.NORMAL : ModuleMode.STEER_ONLY;
            if (m != mode[i]) {
                mode[i] = m;
                changed = true;
            }
            // Keep commanding what can still help; coast only what could do harm. A motor that
            // has taken its config keeps its real command even when we cannot hear it.
            boolean steerCmd = h.steerConfigured[i] && absoluteKnown[i];
            boolean driveCmd = h.driveConfigured[i] && steerUsable;
            changed |= set(cmdSteer, i, steerCmd);
            changed |= set(cmdDrive, i, driveCmd);
            changed |= set(cmdSteerSlow, i, !h.steerConnected[i]);
            changed |= set(cmdDriveSlow, i, !h.driveConnected[i]);
            modeNames[i] = m.name();
            trusted[i] = m == ModuleMode.NORMAL && encoder && h.fusedOk[i];
        }
        if (changed) {
            version++;
            commands =
                    new ModuleCommands(
                            cmdDrive.clone(),
                            cmdSteer.clone(),
                            cmdDriveSlow.clone(),
                            cmdSteerSlow.clone());
        }
        // What each module can still do: NORMAL, STEER_ONLY (drive out), COAST (steer out).
        Logger.recordOutput(prefix + "/Mode", modeNames);
        // Modules odometry may use (health only; each sample also needs fresh signals).
        Logger.recordOutput(prefix + "/Trusted", trusted);
        // Each module's absolute angle has been good at least once since boot.
        Logger.recordOutput(prefix + "/AbsoluteAngleKnown", absoluteKnown);
    }

    /** Per-module weights for one sample: 1 if trusted and fresh, else 0. Reused array. */
    public double[] weights(SwerveSample s) {
        for (int i = 0; i < n; i++) {
            weight[i] = trusted[i] && s.moduleOk[i] ? 1.0 : 0.0;
        }
        return weight;
    }

    /** A copy of each module's mode, for the swerve thread. */
    public ModuleMode[] modes() {
        return mode.clone();
    }

    /** Goes up by one whenever any module's mode or commands change. */
    public int version() {
        return version;
    }

    /** Modules not in {@link ModuleMode#NORMAL}, for alerts; empty when all are. */
    public String degradedSummary(String[] names) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < n; i++) {
            if (mode[i] != ModuleMode.NORMAL) {
                if (sb.length() > 0) {
                    sb.append(", ");
                }
                sb.append(names[i]).append(' ').append(mode[i]);
            }
        }
        return sb.toString();
    }
}
