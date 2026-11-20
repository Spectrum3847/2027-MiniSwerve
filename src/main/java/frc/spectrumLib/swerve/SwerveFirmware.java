package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Each swerve device's firmware version, read once at boot ("major.minor.bugfix.build"). A steer
 * motor and its CANcoder on firmware that does not match each other or the Phoenix library can
 * mistrust the fused sensor's data, so the versions are logged inputs.
 *
 * <p>Blocking (waits for each device's version frames): boot only, before {@code
 * optimizeBusUtilization} turns those frames off.
 */
final class SwerveFirmware {
    private SwerveFirmware() {}

    /** How long to wait for one device's version (s). A missing device costs this once at boot. */
    private static final double TIMEOUT_S = 0.25;

    static String of(TalonFX d) {
        return of(
                d.getVersionMajor(),
                d.getVersionMinor(),
                d.getVersionBugfix(),
                d.getVersionBuild());
    }

    static String of(CANcoder d) {
        return of(
                d.getVersionMajor(),
                d.getVersionMinor(),
                d.getVersionBugfix(),
                d.getVersionBuild());
    }

    static String of(Pigeon2 d) {
        return of(
                d.getVersionMajor(),
                d.getVersionMinor(),
                d.getVersionBugfix(),
                d.getVersionBuild());
    }

    private static String of(
            StatusSignal<Integer> major,
            StatusSignal<Integer> minor,
            StatusSignal<Integer> bugfix,
            StatusSignal<Integer> build) {
        StatusCode code = BaseStatusSignal.waitForAll(TIMEOUT_S, major, minor, bugfix, build);
        if (!code.isOK()) {
            return "unknown (" + code.getName() + ")";
        }
        return major.getValue()
                + "."
                + minor.getValue()
                + "."
                + bugfix.getValue()
                + "."
                + build.getValue();
    }
}
