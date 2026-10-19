package tunnelsim.sensor;

import tunnelsim.scenario.SensorsConfig;
import tunnelsim.scenario.SensorsConfig.ImuConfig;
import tunnelsim.sensor.SensorEvent.ImuSample;
import tunnelsim.util.MathUtil;
import tunnelsim.util.Rng;
import tunnelsim.util.Rot;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Accelerometer and gyroscope. True specific force and angular rate are derived from the truth
 * trajectory in the car body frame, rotated into the phone frame through the (unknown to the
 * estimators) mount rotation, then corrupted by scale/cross-axis errors, bias (power-on, random
 * walk, temperature), vibration, white noise and quantisation.
 */
public final class ImuModel {
    private final SensorsConfig cfg;
    private final double quality;
    private final Rng rng;

    public ImuModel(SensorsConfig cfg, double quality, Rng rng) {
        this.cfg = cfg;
        this.quality = quality;
        this.rng = rng;
    }

    public List<ImuSample> generate(TruthTrajectory truth) {
        List<ImuSample> out = new ArrayList<>();
        int step = Math.max(1, (int) Math.round(1.0 / (cfg.accel.rate * truth.dt)));
        double dt = step * truth.dt;

        AxisErrors acc = new AxisErrors(cfg.accel, quality, rng);
        AxisErrors gyr = new AxisErrors(cfg.gyro, quality, rng);
        double idlePhase = rng.uniform(0, 2 * Math.PI);
        double tunnelTempTarget = rng.uniform(-5, 5);
        double temp = 0;
        SensorsConfig.VibrationConfig vib = cfg.vibration;

        for (int i = 0; i < truth.size(); i += step) {
            double t = truth.t(i);
            double[] f = specificForceBody(truth, i);
            double[] w = angularRateBody(truth, i);

            // Vibration in the body frame: mostly vertical road noise while moving, engine idle when stopped.
            double v = truth.v(i);
            double[] vibAcc;
            if (v > 0.05) {
                double sigma = vib.roadBase + vib.roadPerSpeed * v;
                vibAcc = new double[]{rng.gaussian(0.5 * sigma), rng.gaussian(0.5 * sigma), rng.gaussian(sigma)};
            } else {
                double idle = vib.idleAmplitude * Math.sin(2 * Math.PI * vib.idleFrequency * t + idlePhase);
                vibAcc = new double[]{0.3 * idle + rng.gaussian(0.002), rng.gaussian(0.002), idle + rng.gaussian(0.002)};
            }
            for (int k = 0; k < 3; k++) {
                f[k] += vibAcc[k];
                w[k] += vib.gyroCoupling * vibAcc[(k + 1) % 3];
            }

            // Phone temperature drifts towards the tunnel air temperature.
            double target = truth.tunnel(i) >= 0 ? tunnelTempTarget : 0;
            temp += (target - temp) * dt / 120.0;

            double[] mount = truth.mountAt(t);
            double[] fp = Rot.apply(mount, f);
            double[] wp = Rot.apply(mount, w);
            double[] wRel = mountRate(truth, t, truth.dt);
            for (int k = 0; k < 3; k++) {
                wp[k] += wRel[k];
            }
            double[] am = acc.measure(fp, temp, dt);
            double[] gm = gyr.measure(wp, temp, dt);
            out.add(new ImuSample(t, am[0], am[1], am[2], gm[0], gm[1], gm[2]));
        }
        return out;
    }

    /** True specific force in the body frame [m/s^2]: acceleration minus gravity. */
    public static double[] specificForceBody(TruthTrajectory tr, int i) {
        int i0 = Math.max(0, i - 1), i1 = Math.min(tr.size() - 1, i + 1);
        double[] a = new double[3];
        if (i1 > i0) {
            double[] v0 = velocityNav(tr, i0), v1 = velocityNav(tr, i1);
            double span = tr.t(i1) - tr.t(i0);
            for (int k = 0; k < 3; k++) {
                a[k] = (v1[k] - v0[k]) / span;
            }
        }
        a[2] += MathUtil.G;
        double[] rnb = Rot.bodyToNav(tr.heading(i), tr.pitch(i), tr.roll(i));
        return Rot.apply(Rot.transpose(rnb), a);
    }

    /** True angular rate in the body frame [rad/s], from the change of attitude. */
    public static double[] angularRateBody(TruthTrajectory tr, int i) {
        int i0 = Math.max(0, i - 1), i1 = Math.min(tr.size() - 1, i + 1);
        if (i1 == i0) {
            return new double[3];
        }
        double[] r0 = Rot.bodyToNav(tr.heading(i0), tr.pitch(i0), tr.roll(i0));
        double[] r1 = Rot.bodyToNav(tr.heading(i1), tr.pitch(i1), tr.roll(i1));
        double[] rel = Rot.mul(Rot.transpose(r0), r1);
        double span = tr.t(i1) - tr.t(i0);
        return new double[]{
            (rel[7] - rel[5]) / (2 * span),
            (rel[2] - rel[6]) / (2 * span),
            (rel[3] - rel[1]) / (2 * span)};
    }

    /**
     * Angular rate of the phone relative to the car, in the phone frame, while the phone moves in
     * its mount. With v_phone = M v_body, a phone rotating at w sees dM/dt = -[w]x M.
     * The mount is stored as steps at sample times; differencing over (t - dt/2, t + dt/2] counts
     * each step exactly once, so the integrated rate equals the total rotation. (A central difference
     * over t +- dt, combined with floating-point step times, counted some steps twice and others
     * not at all: the simulated gyro under-reported up to ~20% of a phone shift.)
     */
    private static double[] mountRate(TruthTrajectory tr, double t, double dt) {
        if (tr.mountChanges().size() < 2) {
            return new double[3];
        }
        double[] m0 = tr.mountAt(t - 0.5 * dt), m1 = tr.mountAt(t + 0.5 * dt), m = tr.mountAt(t);
        double[] dm = new double[9];
        for (int k = 0; k < 9; k++) {
            dm[k] = -(m1[k] - m0[k]) / dt;
        }
        double[] w = Rot.mul(dm, Rot.transpose(m));
        return new double[]{(w[7] - w[5]) / 2, (w[2] - w[6]) / 2, (w[3] - w[1]) / 2};
    }

    private static double[] velocityNav(TruthTrajectory tr, int i) {
        double g = Math.atan(tr.grade(i));
        double h = tr.heading(i);
        double v = tr.v(i);
        return new double[]{v * Math.cos(h) * Math.cos(g), v * Math.sin(h) * Math.cos(g), v * Math.sin(g)};
    }

    /** Per-device error state of a 3-axis sensor. */
    private static final class AxisErrors {
        private final ImuConfig c;
        private final double quality;
        private final Rng rng;
        private final double[] m = new double[9];
        private final double[] bias = new double[3];
        private final double[] tempSign = new double[3];

        AxisErrors(ImuConfig c, double quality, Rng rng) {
            this.c = c;
            this.quality = quality;
            this.rng = rng;
            for (int r = 0; r < 3; r++) {
                for (int k = 0; k < 3; k++) {
                    m[3 * r + k] = r == k ? 1 + rng.gaussian(c.scaleSigma) : rng.gaussian(c.crossAxisSigma);
                }
                bias[r] = rng.gaussian(c.biasSigma * quality);
                tempSign[r] = rng.chance(0.5) ? 1 : -1;
            }
        }

        double[] measure(double[] truth, double temp, double dt) {
            double[] out = Rot.apply(m, truth);
            for (int k = 0; k < 3; k++) {
                bias[k] += rng.gaussian(c.biasWalk * quality * Math.sqrt(dt));
                double val = out[k] + bias[k] + tempSign[k] * c.tempCoeff * temp + rng.gaussian(c.noise * quality);
                out[k] = c.quantization > 0 ? Math.round(val / c.quantization) * c.quantization : val;
            }
            return out;
        }
    }
}
