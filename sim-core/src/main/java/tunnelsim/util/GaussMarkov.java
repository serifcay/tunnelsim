package tunnelsim.util;

/**
 * First-order Gauss-Markov process: exponentially correlated noise with a given steady-state
 * standard deviation and correlation time. Used for GNSS position errors, bias drift,
 * barometric disturbances and BLE shadowing, where errors are correlated in time rather than white.
 */
public final class GaussMarkov {
    private final double sigma;
    private final double tau;
    private final Rng rng;
    private double value;

    public GaussMarkov(double sigma, double tau, Rng rng) {
        this.sigma = sigma;
        this.tau = tau;
        this.rng = rng;
        this.value = rng.gaussian(sigma);
    }

    public double step(double dt) {
        if (tau <= 0) {
            value = rng.gaussian(sigma);
            return value;
        }
        double phi = Math.exp(-dt / tau);
        value = phi * value + rng.gaussian(sigma * Math.sqrt(1 - phi * phi));
        return value;
    }

    public double value() {
        return value;
    }
}
