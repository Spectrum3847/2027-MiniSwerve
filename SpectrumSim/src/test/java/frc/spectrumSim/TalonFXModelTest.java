package frc.spectrumSim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.junit.jupiter.api.Test;

class TalonFXModelTest {
    private static final double DT = 0.001;
    private static final MotorModel MOTOR = MotorModel.KRAKEN_X60_FOC;

    /** A flywheel on the motor (rotor rotations and rps), stepped with the model's current. */
    private static final class Flywheel {
        double rot;
        double rps;
        final double inertia;

        Flywheel(double inertia) {
            this.inertia = inertia;
        }

        void step(TalonFXModel m, double dt) {
            double amps = m.update(rot, rps, 12, dt);
            rps += MOTOR.kT() * amps / inertia * dt / (2 * Math.PI);
            rot += rps * dt;
        }
    }

    private static TalonFXModel model(TalonFXConfiguration c) {
        var m = new TalonFXModel("test", MOTOR);
        m.configure(c);
        m.setEnabled(true);
        return m;
    }

    @Test
    void velocityLoopReachesTheTarget() {
        var c = new TalonFXConfiguration();
        c.Feedback.SensorToMechanismRatio = 5;
        c.Slot0.kP = 5;
        var m = model(c);
        var f = new Flywheel(0.001);
        m.setControl(new VelocityTorqueCurrentFOC(10));
        for (int k = 0; k < 2000; k++) {
            f.step(m, DT);
        }
        assertEquals(10, m.velocity(), 0.05);
        assertEquals(50, f.rps, 0.3); // rotor turns SensorToMechanismRatio times faster
    }

    @Test
    void continuousWrapTakesTheShortWayRound() {
        var c = new TalonFXConfiguration();
        c.ClosedLoopGeneral.ContinuousWrap = true;
        c.Slot0.kP = 50;
        c.Slot0.kD = 2;
        var m = model(c);
        var f = new Flywheel(0.0005);
        f.rot = 0.9;
        m.setControl(new PositionTorqueCurrentFOC(0.1));
        for (int k = 0; k < 3000; k++) {
            f.step(m, DT);
        }
        // 0.9 to 0.1 is 0.2 forward through 1.0, not 0.8 back.
        assertEquals(1.1, f.rot, 0.01);
    }

    @Test
    void torqueOutputStopsAtTheStatorLimit() {
        var c = new TalonFXConfiguration();
        c.Slot0.kP = 1000;
        c.CurrentLimits.StatorCurrentLimit = 40;
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        var m = model(c);
        m.setControl(new PositionTorqueCurrentFOC(10));
        assertEquals(40, m.update(0, 0, 12, DT), 1e-9);
    }

    @Test
    void disabledMeansNeutralAndBrakeOpposesMotion() {
        var c = new TalonFXConfiguration();
        c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        c.Slot0.kP = 5;
        var m = model(c);
        m.setControl(new VelocityTorqueCurrentFOC(10));
        m.setEnabled(false);
        assertTrue(m.update(0, 20, 12, DT) < 0, "brake current opposes a turning rotor");
        m.setNeutralMode(NeutralModeValue.Coast);
        assertEquals(0, m.update(0, 20, 12, DT));
        m.setControl(new NeutralOut());
        m.setEnabled(true);
        assertEquals(0, m.update(0, 20, 12, DT));
    }

    @Test
    void dutyCycleScalesWithTheBattery() {
        var m = model(new TalonFXConfiguration());
        m.setControl(new DutyCycleOut(0.5));
        m.update(0, 0, 10, DT);
        assertEquals(5, m.motorVoltage(), 1e-9);
    }

    @Test
    void anUnsupportedRequestCoastsAndIsNamed() {
        var m = model(new TalonFXConfiguration());
        m.setControl(new Follower(1, MotorAlignmentValue.Aligned));
        assertEquals(0, m.update(0, 10, 12, DT));
        assertNotNull(m.unsupportedRequest());
    }

    @Test
    void motionMagicExpoArrivesWithoutRunningPast() {
        var c = new TalonFXConfiguration();
        c.Slot0.kP = 200;
        c.Slot0.kD = 5;
        c.MotionMagic.MotionMagicExpo_kV = 0.12;
        c.MotionMagic.MotionMagicExpo_kA = 0.1;
        var m = model(c);
        var f = new Flywheel(0.0005);
        m.setControl(new MotionMagicExpoTorqueCurrentFOC(3));
        double most = 0;
        for (int k = 0; k < 3000; k++) {
            f.step(m, DT);
            most = Math.max(most, f.rot);
        }
        assertEquals(3, f.rot, 0.01);
        assertTrue(most < 3.2, "overshoot " + most);
    }

    @Test
    void expoProfileTopSpeedIsTwelveVoltsOverKv() {
        var p = new MotionMagicExpoProfile();
        p.reset(0, 0);
        double fastest = 0;
        for (int k = 0; k < 20000; k++) {
            p.step(1000, 2.0, 0.5, 0, false, DT);
            fastest = Math.max(fastest, p.velocity());
        }
        assertEquals(6.0, fastest, 0.05);
    }

    @Test
    void expoProfileLandsOnTheGoal() {
        var p = new MotionMagicExpoProfile();
        p.reset(0, 0);
        double most = 0;
        for (int k = 0; k < 5000; k++) {
            p.step(2, 2.0, 0.5, 0, false, DT);
            most = Math.max(most, p.position());
        }
        assertEquals(2, p.position(), 1e-9);
        assertEquals(0, p.velocity(), 1e-9);
        assertTrue(most <= 2 + 1e-9);
    }
}
