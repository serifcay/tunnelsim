package tunnelsim.util;

public final class MathUtil {
    public static final double G = 9.80665;

    private MathUtil() {
    }

    public static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    public static double wrapAngle(double a) {
        while (a > Math.PI) {
            a -= 2 * Math.PI;
        }
        while (a < -Math.PI) {
            a += 2 * Math.PI;
        }
        return a;
    }

    public static double lerp(double a, double b, double f) {
        return a + (b - a) * f;
    }

    /** Standard normal cumulative distribution function (Abramowitz-Stegun 7.1.26). */
    public static double normalCdf(double z) {
        double t = 1.0 / (1.0 + 0.3275911 * Math.abs(z) / Math.sqrt(2));
        double y = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
                + 0.254829592) * t * Math.exp(-z * z / 2);
        return z >= 0 ? 0.5 * (1 + y) : 0.5 * (1 - y);
    }

    /** Barometric altitude [m] from pressure [Pa] using the standard atmosphere. */
    public static double pressureToAltitude(double pressurePa) {
        return 44330.0 * (1.0 - Math.pow(pressurePa / 101325.0, 1.0 / 5.255));
    }

    /** Pressure [Pa] at altitude [m] using the standard atmosphere. */
    public static double altitudeToPressure(double altitudeM) {
        return 101325.0 * Math.pow(1.0 - altitudeM / 44330.0, 5.255);
    }
}
