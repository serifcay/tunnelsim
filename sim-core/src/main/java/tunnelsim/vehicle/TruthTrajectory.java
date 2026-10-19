package tunnelsim.vehicle;

import tunnelsim.map.MapPosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Ground truth of one car, sampled at a fixed step. Stored as parallel arrays to keep long
 * runs (tens of thousands of samples, several cars) compact.
 */
public final class TruthTrajectory {
    public final double dt;
    /** Leaf edge of the path the car actually drives. */
    public final int leaf;
    private int n;
    private double[] t, offset, s, x, y, z, heading, pitch, roll, v, a, curvature, grade;
    private int[] edge, tunnel;
    private final List<MountChange> mountChanges = new ArrayList<>();

    /** The phone's body-to-phone rotation from time {@code t} onwards. */
    public record MountChange(double t, double[] bodyToPhone) {
    }

    public TruthTrajectory(double dt, int leaf, int capacity) {
        this.dt = dt;
        this.leaf = leaf;
        t = new double[capacity];
        offset = new double[capacity];
        s = new double[capacity];
        x = new double[capacity];
        y = new double[capacity];
        z = new double[capacity];
        heading = new double[capacity];
        pitch = new double[capacity];
        roll = new double[capacity];
        v = new double[capacity];
        a = new double[capacity];
        curvature = new double[capacity];
        grade = new double[capacity];
        edge = new int[capacity];
        tunnel = new int[capacity];
    }

    void add(double tt, int e, double ss, double off, double xx, double yy, double zz, double h, double p, double r,
             double vv, double aa, double k, double g, int tun) {
        if (n == t.length) {
            grow();
        }
        t[n] = tt;
        edge[n] = e;
        s[n] = ss;
        offset[n] = off;
        x[n] = xx;
        y[n] = yy;
        z[n] = zz;
        heading[n] = h;
        pitch[n] = p;
        roll[n] = r;
        v[n] = vv;
        a[n] = aa;
        curvature[n] = k;
        grade[n] = g;
        tunnel[n] = tun;
        n++;
    }

    void addMountChange(MountChange change) {
        mountChanges.add(change);
    }

    private void grow() {
        int cap = t.length * 2;
        t = Arrays.copyOf(t, cap);
        offset = Arrays.copyOf(offset, cap);
        s = Arrays.copyOf(s, cap);
        x = Arrays.copyOf(x, cap);
        y = Arrays.copyOf(y, cap);
        z = Arrays.copyOf(z, cap);
        heading = Arrays.copyOf(heading, cap);
        pitch = Arrays.copyOf(pitch, cap);
        roll = Arrays.copyOf(roll, cap);
        v = Arrays.copyOf(v, cap);
        a = Arrays.copyOf(a, cap);
        curvature = Arrays.copyOf(curvature, cap);
        grade = Arrays.copyOf(grade, cap);
        edge = Arrays.copyOf(edge, cap);
        tunnel = Arrays.copyOf(tunnel, cap);
    }

    public int size() {
        return n;
    }

    public double duration() {
        return n == 0 ? 0 : t[n - 1];
    }

    /** Index of the sample at or just before time {@code time}. */
    public int indexAt(double time) {
        int i = (int) Math.floor(time / dt + 1e-9);
        return Math.max(0, Math.min(n - 1, i));
    }

    public double t(int i) {
        return t[i];
    }

    public int edge(int i) {
        return edge[i];
    }

    public double s(int i) {
        return s[i];
    }

    public MapPosition position(int i) {
        return new MapPosition(edge[i], s[i]);
    }

    public double offset(int i) {
        return offset[i];
    }

    public double x(int i) {
        return x[i];
    }

    public double y(int i) {
        return y[i];
    }

    public double z(int i) {
        return z[i];
    }

    public double heading(int i) {
        return heading[i];
    }

    public double pitch(int i) {
        return pitch[i];
    }

    public double roll(int i) {
        return roll[i];
    }

    public double v(int i) {
        return v[i];
    }

    public double a(int i) {
        return a[i];
    }

    public double curvature(int i) {
        return curvature[i];
    }

    public double grade(int i) {
        return grade[i];
    }

    /** Index of the tunnel the car is in, or -1. */
    public int tunnel(int i) {
        return tunnel[i];
    }

    public List<MountChange> mountChanges() {
        return mountChanges;
    }

    public double[] mountAt(double time) {
        double[] r = mountChanges.get(0).bodyToPhone();
        for (MountChange c : mountChanges) {
            // Tolerance: change times and sample times are both multiples of dt, computed differently.
            if (c.t() <= time + 1e-9) {
                r = c.bodyToPhone();
            }
        }
        return r;
    }
}
