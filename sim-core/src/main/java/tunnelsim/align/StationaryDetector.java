package tunnelsim.align;

/**
 * Detects a stopped car from the accelerometer: a moving car shakes broadband (road vibration
 * grows with speed), a stopped car only shows the weak engine-idle vibration. Enters the stopped
 * state on a low standard deviation of the specific-force magnitude over a sliding window.
 * Leaves it immediately when the (lightly filtered) specific-force vector moves away from its
 * value at rest: pulling away changes the vector long before road vibration builds up, and a
 * late release would tell the estimators "stopped" while the car is already accelerating.
 */
public final class StationaryDetector {
    private static final int WINDOW = 50;          // samples (0.5 s at 100 Hz)
    private static final double ENTER = 0.028;     // m/s^2
    private static final double EXIT = 0.035;      // m/s^2
    private static final double DEPART = 0.15;     // m/s^2 change of the specific-force vector
    private static final double MAX_RATE = 0.03;   // rad/s

    private final double[] buf = new double[WINDOW];
    private int count;
    private int idx;
    private double sum;
    private double sumSq;
    private boolean stationary;
    private final double[] fast = new double[3];
    private final double[] ref = new double[3];
    private boolean fastInit;

    public boolean update(double fx, double fy, double fz, double wx, double wy, double wz) {
        double m = Math.sqrt(fx * fx + fy * fy + fz * fz);
        if (count == WINDOW) {
            double old = buf[idx];
            sum -= old;
            sumSq -= old * old;
        } else {
            count++;
        }
        buf[idx] = m;
        sum += m;
        sumSq += m * m;
        idx = (idx + 1) % WINDOW;

        double[] f = {fx, fy, fz};
        for (int k = 0; k < 3; k++) {
            fast[k] = fastInit ? fast[k] + (f[k] - fast[k]) * 0.2 : f[k]; // ~0.05 s time constant
        }
        fastInit = true;
        if (count < WINDOW) {
            return stationary;
        }
        double mean = sum / count;
        double std = Math.sqrt(Math.max(0, sumSq / count - mean * mean));
        double rate = Math.sqrt(wx * wx + wy * wy + wz * wz);
        if (stationary) {
            double dx = fast[0] - ref[0], dy = fast[1] - ref[1], dz = fast[2] - ref[2];
            boolean departed = Math.sqrt(dx * dx + dy * dy + dz * dz) > DEPART;
            stationary = !departed && std < EXIT && rate < 2 * MAX_RATE;
            if (stationary) {
                for (int k = 0; k < 3; k++) {
                    ref[k] += (fast[k] - ref[k]) * 0.002;
                }
            }
        } else if (std < ENTER && rate < MAX_RATE) {
            stationary = true;
            System.arraycopy(fast, 0, ref, 0, 3);
        }
        return stationary;
    }

    public boolean stationary() {
        return stationary;
    }
}
