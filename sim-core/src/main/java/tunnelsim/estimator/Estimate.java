package tunnelsim.estimator;

/**
 * An estimator's belief at one instant.
 *
 * @param leaf      leaf edge of the path the position refers to
 * @param offset    path offset of the position along that path [m]
 * @param sigma     1-sigma along-road uncertainty [m]
 * @param pOffRoute probability that the car is no longer on the planned route
 * @param particles subsample of particles for display, or null
 */
public record Estimate(double t, boolean valid, int edge, double s, int leaf, double offset, double sigma,
                       double speed, double sigmaSpeed, double pOffRoute, Mode mode, ParticleSnapshot particles) {

    public enum Mode { INIT, GNSS, DEAD_RECKONING, RECOVERING }

    public record ParticleSnapshot(int[] edges, float[] s, float[] weights) {
    }

    public static Estimate invalid(double t) {
        return new Estimate(t, false, 0, 0, 0, 0, Double.NaN, 0, 0, 0, Mode.INIT, null);
    }
}
