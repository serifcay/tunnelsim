package tunnelsim.scenario;

/** Parameters shared by the estimators, including the per-aiding on/off switches. */
public class EstimatorConfig {
    /** ESTIMATED or TRUTH (use the true phone mount, to isolate estimator errors). */
    public String alignment = "ESTIMATED";
    public int particles = 1000;
    /** Prior probability that the car follows the planned route at a fork. */
    public double pFollow = 0.8;
    /** Minimum share of particles kept on each branch shortly after a fork. */
    public double branchMinShare = 0.05;
    public double branchGuardTime = 8;
    public AidingConfig aiding = new AidingConfig();

    public static class AidingConfig {
        public boolean gnss = true;
        public boolean curvature = true;
        public boolean lateralAccel = true;
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
