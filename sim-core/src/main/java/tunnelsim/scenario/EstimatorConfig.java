package tunnelsim.scenario;

/** Parameters shared by the estimators, including the per-aiding on/off switches. */
public class EstimatorConfig {
    /** ESTIMATED or TRUTH (use the true phone mount, to isolate estimator errors). */
    public String alignment = "ESTIMATED";
    /** Apply the effective forward accelerometer scale learned by the alignment front-end. */
    public boolean accelScaleCalibration = true;
    public int particles = 1000;
    /** Prior probability that the car follows the planned route at a fork. */
    public double pFollow = 0.8;
    /** Minimum share of particles kept on each branch shortly after a fork. */
    public double branchMinShare = 0.05;
    public double branchGuardTime = 8;
    public AidingConfig aiding = new AidingConfig();
    public ParticleFilterConfig pf = new ParticleFilterConfig();

    /** Particle filter noise model (D). */
    public static class ParticleFilterConfig {
        /** Speed process noise [m/s per sqrt(s)]. */
        public double sigmaV = 0.15;
        /** Accelerometer bias random walk [m/s^2 per sqrt(s)]. */
        public double sigmaBias = 0.002;
        /** Yaw-rate likelihood: absolute [rad/s] plus relative part (gyro scale, map curvature error). */
        public double sigmaYaw = 0.004;
        public double sigmaYawRelative = 0.03;
        /** Lateral-acceleration likelihood: absolute [m/s^2] plus relative part (body roll). */
        public double sigmaLat = 0.15;
        public double sigmaLatRelative = 0.15;
        /** Barometric altitude likelihood [m]. */
        public double sigmaBaro = 0.8;
        /**
         * Correlation time of in-tunnel pressure disturbances (piston effect, ventilation) [s].
         * Barometer updates are down-weighted accordingly; treating them as independent made the
         * filter several times over-confident.
         */
        public double baroCorrelationTime = 20;
        /** Barometer update interval [s]. */
        public double baroInterval = 1.0;
        /** Curvature matching window [s]; one update per window. */
        public double curvatureWindow = 1.0;
        /**
         * Prior std-dev of the gyro scale factor error, carried per particle. A scale error biases
         * the speed inferred from curvature matching consistently, so it cannot be treated as noise.
         */
        public double gyroScaleSigma = 0.01;
    }

    public static class AidingConfig {
        public boolean gnss = true;
        public boolean curvature = true;
        /** Off by default: body roll in curves biases lateral acceleration by several percent. */
        public boolean lateralAccel = false;
        public boolean baro = true;
        public boolean zupt = true;
        public boolean speedPrior = true;
        public boolean portal = true;
        public boolean cell = true;
        /** Only effective when options.ble is on. */
        public boolean ble = true;
        /** Only effective when options.obd is on. */
        public boolean obd = true;
    }
}
