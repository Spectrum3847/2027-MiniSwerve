package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.spectrumLib.hardware.PhoenixDeviceSet;
import frc.spectrumLib.hardware.SimFaults;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongConsumer;
import org.wpilib.system.RobotController;

/**
 * Our swerve on plain Phoenix 6 Pro devices (no CTRE swerve API). Each module's control runs on the
 * motors at 1 kHz: steer with PositionTorqueCurrentFOC on the fused CANcoder, drive with
 * VelocityTorqueCurrentFOC. The 250 Hz thread only sends setpoints and reads latency-compensated
 * signals.
 *
 * <p>The desktop sim normally uses SpectrumSim's {@code SwerveIOSim} (TalonFX models, repeatable).
 * This IO also runs in sim on Phoenix's own simulator, to check that path and SpectrumSim's models
 * against CTRE's: SpectrumSim reaches the devices through {@link #simDevices} and paces the reads
 * with {@link #setSimPacer}.
 *
 * <p>Device configs are in {@link SwervePhoenixConfigs}; reboot detection and endless config
 * retries in {@code PhoenixDeviceSet}.
 *
 * <p>Index: constructor | {@link #waitForSample} | {@link #setModules} | {@link
 * #retryFailedConfigs} | {@link #setDriveBrake} | {@link #updateHealth}
 */
public final class SwerveIOPhoenix implements SwerveIO {
    private final PhoenixSwerveConfig cfg;
    private final int n;
    private final TalonFX[] drive;
    private final TalonFX[] steer;
    private final CANcoder[] encoder;
    private final Pigeon2 pigeon;

    private final StatusSignal<?>[] drivePosition;
    private final StatusSignal<?>[] driveVelocity;
    private final StatusSignal<?>[] steerPosition;
    private final StatusSignal<?>[] steerVelocity;
    private final StatusSignal<?> yaw;
    private final StatusSignal<?> yawRate;
    private final BaseStatusSignal[] fast;
    private final BaseStatusSignal[] slow;
    private final PacedSignals paced;

    private final double metersPerWheelRotation;

    /** Desktop sim only: waits out the rest of a period after a read, from its start (ns). */
    private volatile LongConsumer simPacer;

    private final SwerveRequests requests;
    private final double[] measuredSteerRps;
    private final SwerveRequests.Motors motors =
            new SwerveRequests.Motors() {
                @Override
                public void setDrive(int i, ControlRequest r) {
                    drive[i].setControl(r);
                }

                @Override
                public void setSteer(int i, ControlRequest r) {
                    steer[i].setControl(r);
                }
            };

    /** Keeps every device configured for the match: reboots noticed, configs retried. */
    private final Pigeon2Configuration pigeonConfig;

    private final boolean timesync;
    private final DriveNeutralMode neutralMode;

    private final SimFaults faults;

    /** Device names (as in PhoenixDeviceSet), built once: no strings per cycle. */
    private final String[] driveName;

    private final String[] steerName;

    private final PhoenixDeviceSet devices = new PhoenixDeviceSet("Swerve");

    private final int[] driveIndex;
    private final int[] steerIndex;
    private final int[] encoderIndex;
    private final int pigeonIndex;

    /**
     * Builds and configures every device. Blocking (config applies): call during robot init only.
     *
     * @param faults simulated device faults (sim only; null on the robot)
     */
    public SwerveIOPhoenix(PhoenixSwerveConfig cfg, SimFaults faults) {
        this.cfg = cfg;
        this.faults = faults;
        devices.setSimFaults(faults);
        n = cfg.modules();
        CANBus bus = new CANBus(cfg.canBus());
        // CANivore (CAN FD) with Pro: Phoenix timesync lines up every device's frames and applies
        // setpoints at the same instant. SystemCore's own bus does not support it.
        timesync = bus.isNetworkFD() && cfg.allowTimesync();
        requests = new SwerveRequests(cfg, faults);
        requests.setTimesync(timesync);
        measuredSteerRps = new double[n];
        drive = new TalonFX[n];
        neutralMode = new DriveNeutralMode(drive);
        steer = new TalonFX[n];
        encoder = new CANcoder[n];
        drivePosition = new StatusSignal<?>[n];
        driveVelocity = new StatusSignal<?>[n];
        steerPosition = new StatusSignal<?>[n];
        steerVelocity = new StatusSignal<?>[n];
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        pigeon = new Pigeon2(cfg.pigeonId(), bus);
        driveIndex = new int[n];
        driveName = new String[n];
        steerName = new String[n];
        for (int i = 0; i < n; i++) {
            driveName[i] = "Drive" + i;
            steerName[i] = "Steer" + i;
        }
        steerIndex = new int[n];
        encoderIndex = new int[n];
        pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPoseYaw = cfg.pigeonMountPose()[0];
        pigeonConfig.MountPose.MountPosePitch = cfg.pigeonMountPose()[1];
        pigeonConfig.MountPose.MountPoseRoll = cfg.pigeonMountPose()[2];
        pigeonIndex =
                devices.add("Pigeon", pigeon, () -> pigeon.getConfigurator().apply(pigeonConfig));

        List<BaseStatusSignal> fastList = new ArrayList<>();
        List<BaseStatusSignal> slowList = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            encoder[i] = new CANcoder(cfg.encoderIds()[i], bus);
            drive[i] = new TalonFX(cfg.driveIds()[i], bus);
            steer[i] = new TalonFX(cfg.steerIds()[i], bus);
            final int m = i;
            // Each device separately: one failure only degrades its own part, and is retried.
            encoderIndex[i] =
                    devices.add(
                            "Encoder" + i,
                            encoder[i],
                            () ->
                                    encoder[m]
                                            .getConfigurator()
                                            .apply(SwervePhoenixConfigs.encoder(cfg, m)));
            steerIndex[i] =
                    devices.add(
                            steerName[i],
                            steer[i],
                            () ->
                                    steer[m].getConfigurator()
                                            .apply(SwervePhoenixConfigs.steer(cfg, m)));
            driveIndex[i] = devices.add(driveName[i], drive[i], () -> applyDriveConfig(m));

            drivePosition[i] = drive[i].getPosition();
            driveVelocity[i] = drive[i].getVelocity();
            steerPosition[i] = steer[i].getPosition();
            steerVelocity[i] = steer[i].getVelocity();
            fastList.addAll(
                    List.of(
                            drivePosition[i],
                            driveVelocity[i],
                            steerPosition[i],
                            steerVelocity[i]));
            slowList.addAll(
                    List.of(
                            drive[i].getStatorCurrent(),
                            steer[i].getStatorCurrent(),
                            drive[i].getSupplyCurrent(),
                            steer[i].getSupplyCurrent(),
                            drive[i].getDeviceTemp(),
                            steer[i].getDeviceTemp(),
                            drive[i].getSupplyVoltage(),
                            steer[i].getFault_FusedSensorOutOfSync(),
                            steer[i].getFault_RemoteSensorDataInvalid(),
                            encoder[i].getFault_BadMagnet(),
                            steer[i].getSupplyVoltage()));
        }
        // Tilt for the 3D pose: the quaternion, as Pigeon2.getRotation3d reads it, so the frame is
        // WPILib's by construction (no Euler sign conventions to get wrong).
        slowList.addAll(
                List.of(
                        pigeon.getQuatW(),
                        pigeon.getQuatX(),
                        pigeon.getQuatY(),
                        pigeon.getQuatZ()));
        yaw = pigeon.getYaw();
        yawRate = pigeon.getAngularVelocityZWorld();
        fastList.add(yaw);
        fastList.add(yawRate);
        fast = fastList.toArray(new BaseStatusSignal[0]);
        // One group per device (drive, steer per module, then the Pigeon), so a silent device
        // stops pacing the thread without slowing the others.
        int[][] groups = new int[2 * n + 1][];
        for (int i = 0; i < n; i++) {
            groups[2 * i] = new int[] {4 * i, 4 * i + 1};
            groups[2 * i + 1] = new int[] {4 * i + 2, 4 * i + 3};
        }
        groups[2 * n] = new int[] {4 * n, 4 * n + 1};
        paced = new PacedSignals(fast, groups);
        slow = slowList.toArray(new BaseStatusSignal[0]);
        devices.setSignals(
                () ->
                        SwervePhoenixConfigs.signals(
                                cfg.signalHz(), fast, slow, drive, steer, encoder, pigeon));
        devices.configureAtBoot();
    }

    /** The Phoenix devices, for a desktop sim to move (never used on the robot). */
    public record SimDevices(
            TalonFX[] drive, TalonFX[] steer, CANcoder[] encoder, Pigeon2 pigeon) {}

    public SimDevices simDevices() {
        return new SimDevices(drive, steer, encoder, pigeon);
    }

    /**
     * Desktop sim on Phoenix's simulator: read signals without waiting for new frames, then let
     * {@code pacer} wait out the period (given the read's start, ns). Null on the robot.
     */
    public void setSimPacer(LongConsumer pacer) {
        simPacer = pacer;
    }

    /**
     * Slow signals per module: stator and supply current for both motors, 2 temps, drive voltage, 3
     * faults, steer voltage.
     */
    private static final int SLOW_PER_MODULE = 11;

    /** Applies a drive's whole config with the neutral mode wanted now. */
    private com.ctre.phoenix6.StatusCode applyDriveConfig(int i) {
        boolean brake = neutralMode.wantedBrake();
        return neutralMode.applyFull(i, SwervePhoenixConfigs.drive(cfg, i, brake), brake);
    }

    @Override
    public void waitForSample(SwerveSample out) {
        boolean good;
        LongConsumer pacer = simPacer;
        if (pacer != null) {
            // Desktop: the physics runs on its own thread; read what Phoenix's simulator reports,
            // then wait out the rest of the period.
            long start = System.nanoTime();
            good = BaseStatusSignal.refreshAll(fast).isOK();
            pacer.accept(start);
        } else {
            good = paced.waitForSample(2.0 / cfg.signalHz());
        }
        out.allGood = good;
        // Capture time: the mean over fresh signals of their own timestamps (system receive time on
        // a
        // plain bus, the synchronized CANivore time with timesync), not when we got around to
        // reading them. Both are on Phoenix's clock (Utils.getCurrentTimeSeconds).
        double sum = 0;
        int count = 0;
        for (BaseStatusSignal sig : fast) {
            if (sig.getStatus().isOK()) {
                var all = sig.getAllTimestamps();
                sum += (timesync ? all.getCANivoreTimestamp() : all.getSystemTimestamp()).getTime();
                count++;
            }
        }
        double phoenixNow = Utils.getCurrentTimeSeconds();
        double phoenixTime = count > 0 ? sum / count : phoenixNow;
        // On the robot's clock (RobotController.getMonotonicTime, the camera frames' clock):
        // Phoenix
        // keeps its own, so shift by the two clocks' offset read back to back now. The sample is a
        // logged input, so replay uses the shifted time as recorded.
        out.timestamp = phoenixTime + (RobotController.getMonotonicTime() * 1e-9 - phoenixNow);
        out.gyroOk = yaw.getStatus().isOK() && yawRate.getStatus().isOK();
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        for (int i = 0; i < n; i++) {
            if (faults != null && (unheard(driveName[i]) || unheard(steerName[i]))) {
                // Sim: no frames from this module, so its readings freeze, as on the robot.
                out.moduleOk[i] = false;
                continue;
            }
            double steerRot =
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                            steerPosition[i], steerVelocity[i]);
            double wheelRot =
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                            drivePosition[i], driveVelocity[i]);
            // MK4i: turning the module turns the drive gearing; remove that from wheel travel.
            out.distance[i] = (wheelRot - steerRot * couple) * metersPerWheelRotation;
            out.velocity[i] =
                    (driveVelocity[i].getValueAsDouble()
                                    - steerVelocity[i].getValueAsDouble() * couple)
                            * metersPerWheelRotation;
            out.angle[i] = SwerveMath.angleModulus(steerRot * 2 * Math.PI);
            out.moduleOk[i] =
                    drivePosition[i].getStatus().isOK()
                            && driveVelocity[i].getStatus().isOK()
                            && steerPosition[i].getStatus().isOK()
                            && steerVelocity[i].getStatus().isOK();
        }
        out.yaw = Math.toRadians(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawRate));
        out.angularVelocityZWorld = Math.toRadians(yawRate.getValueAsDouble());
    }

    private boolean unheard(String device) {
        return faults != null && faults.unheard(device);
    }

    @Override
    public void setModules(
            double[] angle,
            double[] velocity,
            double[] steerVelocityPlan,
            double[] wheelForce,
            ModuleCommands commands) {
        for (int i = 0; i < n; i++) {
            measuredSteerRps[i] = steerVelocity[i].getValueAsDouble();
        }
        requests.setModules(
                angle, velocity, steerVelocityPlan, wheelForce, commands, measuredSteerRps, motors);
    }

    @Override
    public void retryFailedConfigs() {
        devices.poll();
    }

    @Override
    public void setDriveBrake(boolean brake) {
        neutralMode.request(brake);
    }

    @Override
    public void setNeutral() {
        requests.setNeutral(motors);
    }

    @Override
    public void updateHealth(SwerveHealth h) {
        BaseStatusSignal.refreshAll(slow);
        for (int i = 0; i < n; i++) {
            int b = i * SLOW_PER_MODULE;
            h.driveConnected[i] = devices.connected(driveIndex[i]);
            h.steerConnected[i] = devices.connected(steerIndex[i]);
            h.encoderConnected[i] = devices.connected(encoderIndex[i]);
            h.driveConfigured[i] = devices.configured(driveIndex[i]);
            h.steerConfigured[i] = devices.configured(steerIndex[i]);
            h.encoderConfigured[i] = devices.configured(encoderIndex[i]);
            h.driveStatorCurrent[i] = slow[b].getValueAsDouble();
            h.steerStatorCurrent[i] = slow[b + 1].getValueAsDouble();
            h.driveSupplyCurrent[i] = slow[b + 2].getValueAsDouble();
            h.steerSupplyCurrent[i] = slow[b + 3].getValueAsDouble();
            h.driveDeviceTemp[i] = slow[b + 4].getValueAsDouble();
            h.steerDeviceTemp[i] = slow[b + 5].getValueAsDouble();
            // Fault signals are Boolean; getValueAsDouble is 1 when the fault is active.
            h.fusedOk[i] =
                    slow[b + 7].getValueAsDouble() == 0 && slow[b + 8].getValueAsDouble() == 0;
            h.magnetOk[i] = slow[b + 9].getValueAsDouble() == 0;
            h.driveSupplyVoltage[i] = slow[b + 6].getValueAsDouble();
            h.steerSupplyVoltage[i] = slow[b + 10].getValueAsDouble();
        }
        // Highest reading among connected drive motors: one unplugged motor must not freeze it.
        double volts = Double.NaN;
        for (int i = 0; i < n; i++) {
            if (h.driveConnected[i]) {
                double v = h.driveSupplyVoltage[i];
                volts = Double.isNaN(volts) ? v : Math.max(volts, v);
            }
        }
        h.supplyVoltage = Double.isNaN(volts) ? 0.0 : volts;
        h.pigeonConnected = devices.connected(pigeonIndex) && devices.configured(pigeonIndex);
        int q = n * SLOW_PER_MODULE;
        if (h.pigeonConnected) {
            var tilt =
                    new org.wpilib.math.geometry.Rotation3d(
                            new org.wpilib.math.geometry.Quaternion(
                                    slow[q].getValueAsDouble(),
                                    slow[q + 1].getValueAsDouble(),
                                    slow[q + 2].getValueAsDouble(),
                                    slow[q + 3].getValueAsDouble()));
            h.roll = tilt.getX();
            h.pitch = tilt.getY();
        } else {
            h.roll = 0;
            h.pitch = 0;
        }
        h.timesync = timesync;
        h.deviceResets = devices.resets();
        h.signalsSet = devices.signalsSet();
        // Brake only if every drive motor is in brake.
        h.driveBrake = neutralMode.allBrake();
    }
}
