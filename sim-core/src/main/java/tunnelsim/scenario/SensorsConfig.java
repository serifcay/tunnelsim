package tunnelsim.scenario;

/**
 * Sensor model parameters. Defaults are ballpark values for a modern mid/high-end phone and
 * still need checking against datasheets (see docs/DESIGN.md §2).
 */
public class SensorsConfig {
    public ImuConfig accel = ImuConfig.accelDefaults();
    public ImuConfig gyro = ImuConfig.gyroDefaults();
    public VibrationConfig vibration = new VibrationConfig();
    public GnssConfig gnss = new GnssConfig();
    public BaroConfig baro = new BaroConfig();
    public LightConfig light = new LightConfig();
    public MagConfig mag = new MagConfig();
    public CellConfig cell = new CellConfig();
    public BleConfig ble = new BleConfig();
    public ObdConfig obd = new ObdConfig();
    /** Std-dev of sensor timestamp jitter [s]. */
    public double timestampJitter = 0;

    public static class ImuConfig {
        public double rate = 100;
        /** White noise std-dev per sample [m/s^2 or rad/s]. */
        public double noise;
        /** Std-dev of the power-on bias per axis. */
        public double biasSigma;
        /** Bias random walk [unit/sqrt(s)]. */
        public double biasWalk;
        /** Std-dev of the scale factor error (fraction). */
        public double scaleSigma = 0.01;
        /** Std-dev of cross-axis coupling (fraction). */
        public double crossAxisSigma = 0.005;
        /** Bias change per degree Celsius per axis (random sign). */
        public double tempCoeff;
        /** Quantisation step (0 disables). */
        public double quantization;

        static ImuConfig accelDefaults() {
            ImuConfig c = new ImuConfig();
            c.noise = 0.012;
            c.biasSigma = 0.15;
            c.biasWalk = 0.0005;
            c.tempCoeff = 0.002;
            c.quantization = 0.0024;
            return c;
        }

        static ImuConfig gyroDefaults() {
            ImuConfig c = new ImuConfig();
            c.noise = Math.toRadians(0.06);
            c.biasSigma = Math.toRadians(0.1);
            c.biasWalk = Math.toRadians(0.002);
            c.tempCoeff = Math.toRadians(0.005);
            c.quantization = Math.toRadians(0.004);
            return c;
        }
    }

    public static class VibrationConfig {
        /** Road vibration std-dev while moving: base + perSpeed * v [m/s^2]. */
        public double roadBase = 0.04;
        public double roadPerSpeed = 0.006;
        /** Engine idle vibration amplitude when stopped [m/s^2]. */
        public double idleAmplitude = 0.015;
        public double idleFrequency = 24;
        /** Gyro vibration as a fraction of accel vibration [rad/s per m/s^2]. */
        public double gyroCoupling = 0.01;
    }

    public static class GnssConfig {
        public double rate = 1;
        /** Horizontal error std-dev per axis in open sky [m]. */
        public double sigma = 2.5;
        /** Correlation time of the position error [s]. */
        public double tau = 40;
        /** Doppler speed std-dev [m/s]. */
        public double speedSigma = 0.15;
        public double latency = 0;
        /** Reported accuracy = true sigma times a factor in this range (below 1 = optimistic). */
        public double[] reportedAccuracyFactor = {0.6, 1.1};
        /** Delay before the first fix after leaving a tunnel [s]. */
        public double[] reacquireDelay = {1, 5};
        /** Multipath bias magnitude right after reacquisition [m] and its decay time [s]. */
        public double[] exitMultipath = {5, 25};
        public double exitMultipathTau = 6;
        /** Fix quality below which no fix is produced (degraded zone). */
        public double minQuality = 0.35;
    }

    public static class BaroConfig {
        public boolean enabled = true;
        public double rate = 25;
        /** White noise [Pa]. */
        public double noise = 3;
        /** Slow weather drift [Pa/sqrt(s)]. */
        public double drift = 0.05;
        /** Tunnel piston-effect disturbance std-dev [Pa] and correlation time [s]. */
        public double tunnelSigma = 4;
        public double tunnelTau = 25;
        /** Pressure step on entering a tunnel, drawn from [-x, x] [Pa], decays with tunnelTau. */
        public double entryStep = 8;
        /** Rate of car ventilation / window spikes [1/s] and their magnitude [Pa]. */
        public double spikeRate = 1.0 / 120;
        public double spikeMagnitude = 15;
    }

    public static class LightConfig {
        public double rate = 5;
        /** Outdoor daylight range at the phone's sensor [lux]. */
        public double[] dayLux = {2000, 20000};
        public double nightLux = 8;
        /** Tunnel interior and entrance-zone lighting [lux]. */
        public double tunnelLux = 120;
        public double entranceLux = 900;
        public double entranceLength = 120;
        /** Modulation depth of the ceiling light pattern (0..1). */
        public double lightModulation = 0.3;
        /** Sensor response time constant [s]. */
        public double lag = 0.3;
        public double noiseFraction = 0.05;
    }

    public static class MagConfig {
        public double rate = 10;
        public double noise = 0.5;
        /** Disturbance std-dev inside tunnels [uT]. */
        public double tunnelDisturbance = 25;
    }

    public static class CellConfig {
        /** Spacing of cell handover zones inside tunnels with in-tunnel coverage [m]. */
        public double handoverSpacing = 1500;
        /** Std-dev of where the handover actually happens [m]. */
        public double jitter = 60;
        public double missProbability = 0.1;
    }

    public static class BleConfig {
        /** Beacon spacing inside tunnels [m]. */
        public double spacing = 100;
        /** RSSI at 1 m [dBm] and path loss exponent. */
        public double txPower = -59;
        public double pathLossExponent = 2.2;
        /** Shadowing std-dev [dB] and correlation time [s]. */
        public double shadowing = 5;
        public double shadowingTau = 1.5;
        public double scanInterval = 1.0;
        public double detectionProbability = 0.85;
        public double sensitivity = -95;
        /** Fraction of beacons that are dead. */
        public double failedFraction = 0;
        /** Std-dev of the difference between surveyed and true beacon position [m]. */
        public double positionError = 2;
        /** Beacon offset from the lane centre and mounting height [m]. */
        public double lateralOffset = 3;
        public double height = 4;
    }

    public static class ObdConfig {
        public double rate = 5;
        /** Speedometer scale error drawn from this range (fraction). */
        public double[] scaleError = {0.01, 0.03};
        /** Quantisation [m/s] (1 km/h). */
        public double quantization = 1 / 3.6;
        public double latency = 0.2;
    }
}
