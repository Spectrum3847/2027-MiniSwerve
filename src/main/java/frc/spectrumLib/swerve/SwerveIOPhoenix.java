package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.spectrumLib.hardware.PhoenixDeviceSet;
import frc.spectrumLib.hardware.SimFaults;
import java.util.ArrayList;
import java.util.List;

/**
 * Our swerve on plain Phoenix 6 Pro devices (no CTRE swerve API). Each module's control runs on the
 * motors at 1 kHz: steer with PositionTorqueCurrentFOC on the fused CANcoder, drive with
 * VelocityTorqueCurrentFOC. The 250 Hz thread only sends setpoints and reads latency-compensated
 * signals.
 *
 * <p>In simulation the same devices run through Phoenix's simulator and {@link SwervePhysicsSim}
 * moves them, so sim exercises the exact control path the robot uses.
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

    // UpdateFreqHz 0: send each setpoint once when we call setControl, as CTRE's swerve does,
    // instead of Phoenix also repeating it on its own 100 Hz timer. Halves control traffic.
    private final VelocityTorqueCurrentFOC driveRequest =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC steerRequest =
            new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final MotionMagicExpoTorqueCurrentFOC steerExpoRequest =
            new MotionMagicExpoTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionVoltage steerVoltageRequest =
            new PositionVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final MotionMagicExpoVoltage steerExpoVoltageRequest =
            new MotionMagicExpoVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final NeutralOut neutral = new NeutralOut().withUpdateFreqHz(0);
    // Once per call too: Phoenix would otherwise resend it at 100 Hz to a dead motor.
    private final CoastOut coast = new CoastOut().withUpdateFreqHz(0);
    private final double metersPerWheelRotation;
    private final SwervePhysicsSim sim;

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
     * @param sim physics to step each cycle, or null on the real robot
     * @param faults simulated device faults (sim only; null on the robot)
     */
    public SwerveIOPhoenix(PhoenixSwerveConfig cfg, SwervePhysicsSim sim, SimFaults faults) {
        this.cfg = cfg;
        this.sim = sim;
        this.faults = faults;
        devices.setSimFaults(faults);
        n = cfg.modules();
        CANBus bus = new CANBus(cfg.canBus());
        // CANivore (CAN FD) with Pro: Phoenix timesync lines up every device's frames and applies
        // setpoints at the same instant. SystemCore's own bus does not support it.
        timesync = bus.isNetworkFD() && cfg.allowTimesync();
        driveRequest.UseTimesync = timesync;
        steerRequest.UseTimesync = timesync;
        steerExpoRequest.UseTimesync = timesync;
        steerVoltageRequest.UseTimesync = timesync;
        steerExpoVoltageRequest.UseTimesync = timesync;
        drive = new TalonFX[n];
        neutralMode = new DriveNeutralMode(drive);
        steer = new TalonFX[n];
        encoder = new CANcoder[n];
        drivePosition = new StatusSignal<?>[n];
        driveVelocity = new StatusSignal<?>[n];
        steerPosition = new StatusSignal<?>[n];
        steerVelocity = new StatusSignal<?>[n];
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        slowDivisor = Math.max(1, (int) Math.round(cfg.signalHz() / 10.0));
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
                            encoder[i].getFault_BadMagnet()));
        }
        yaw = pigeon.getYaw();
        yawRate = pigeon.getAngularVelocityZWorld();
        fastList.add(yaw);
        fastList.add(yawRate);
        fast = fastList.toArray(new BaseStatusSignal[0]);
        slow = slowList.toArray(new BaseStatusSignal[0]);
        devices.setSignals(
                () ->
                        SwervePhoenixConfigs.signals(
                                cfg.signalHz(), fast, slow, drive, steer, encoder, pigeon));
        devices.configureAtBoot();
        if (sim != null) {
            sim.attach(cfg, drive, steer, encoder, pigeon);
        }
    }

    /** Slow signals per module: 3 currents x2 motors, 2 temps, voltage, 3 faults. */
    private static final int SLOW_PER_MODULE = 10;

    /** Applies a drive's whole config with the neutral mode wanted now. */
    private com.ctre.phoenix6.StatusCode applyDriveConfig(int i) {
        boolean brake = neutralMode.wantedBrake();
        return neutralMode.applyFull(i, SwervePhoenixConfigs.drive(cfg, i, brake), brake);
    }

    @Override
    public void waitForSample(SwerveSample out) {
        boolean good;
        if (sim != null) {
            // Desktop: step the physics one period, read what Phoenix's simulator reports, then
            // wait out the rest of the period.
            long start = sim.stepRealTime(cfg.signalHz());
            good = BaseStatusSignal.refreshAll(fast).isOK();
            sim.waitForNextPeriod(start, cfg.signalHz());
        } else {
            good = BaseStatusSignal.waitForAll(2.0 / cfg.signalHz(), fast).isOK();
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
        out.timestamp = count > 0 ? sum / count : Utils.getCurrentTimeSeconds();
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

    /** Sim: a device without power ignores commands. */
    private boolean unpowered(String device) {
        return faults != null && faults.unpowered(device);
    }

    /** Cycles between commands to a disconnected motor: 10 Hz instead of the loop rate. */
    private final int slowDivisor;

    private long cycle;

    @Override
    public void setModules(
            double[] angle,
            double[] velocity,
            double[] steerVelocityPlan,
            double[] wheelForce,
            ModuleCommands commands) {
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        // Wheel force (N) to drive torque current (A): F * r / (ratio * Kt).
        double ampsPerNewton = cfg.wheelRadius() / (cfg.driveRatio() * cfg.driveKt());
        // A disconnected motor still gets its command, but only every slowDivisor cycles: it may
        // be alive and listening, and the rest of the bus should not pay for it every cycle.
        boolean slowTurn = cycle++ % slowDivisor == 0;
        for (int i = 0; i < n; i++) {
            if ((!commands.steerSlow()[i] || slowTurn) && !unpowered(steerName[i])) {
                if (commands.steer()[i]) {
                    double target = angle[i] / (2 * Math.PI);
                    double steerRps = steerVelocityPlan[i] / (2 * Math.PI);
                    // The planned turn rate keeps the steer D term from fighting the motion.
                    if (cfg.steerVoltage()) {
                        steer[i].setControl(
                                cfg.steerMotionMagicExpo()
                                        ? steerExpoVoltageRequest.withPosition(target)
                                        : steerVoltageRequest
                                                .withPosition(target)
                                                .withVelocity(steerRps));
                    } else {
                        steer[i].setControl(
                                cfg.steerMotionMagicExpo()
                                        ? steerExpoRequest.withPosition(target)
                                        : steerRequest.withPosition(target).withVelocity(steerRps));
                    }
                } else {
                    steer[i].setControl(coast);
                }
            }
            if ((!commands.driveSlow()[i] || slowTurn) && !unpowered(driveName[i])) {
                if (commands.drive()[i]) {
                    double wheelRps = velocity[i] / metersPerWheelRotation;
                    // Coupling feedforward from the module's measured turn rate: a commanded rate
                    // would spike whenever the target angle jumps.
                    double measuredSteerRps = steerVelocity[i].getValueAsDouble();
                    drive[i].setControl(
                            driveRequest
                                    .withVelocity(wheelRps + measuredSteerRps * couple)
                                    .withFeedForward(wheelForce[i] * ampsPerNewton));
                } else {
                    drive[i].setControl(coast);
                }
            }
        }
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
        for (int i = 0; i < n; i++) {
            drive[i].setControl(neutral);
            steer[i].setControl(neutral);
        }
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
        }
        h.supplyVoltage = slow[6].getValueAsDouble();
        h.pigeonConnected = devices.connected(pigeonIndex) && devices.configured(pigeonIndex);
        h.timesync = timesync;
        h.deviceResets = devices.resets();
        h.signalsSet = devices.signalsSet();
        // Brake only if every drive motor is in brake.
        h.driveBrake = neutralMode.allBrake();
    }
}
