package tunnelsim.util;

/** Minimal 3x3 rotation / vector helpers (row-major double[9], vectors as double[3]). */
public final class Rot {
    private Rot() {
    }

    /**
     * Body-to-navigation rotation for ZYX Euler angles (yaw about z, then pitch about y, then roll
     * about x). Navigation frame: x east, y north, z up. Body frame: x forward, y left, z up.
     * Positive pitch is nose up, positive roll is right side down.
     */
    public static double[] bodyToNav(double yaw, double pitch, double roll) {
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cr = Math.cos(roll), sr = Math.sin(roll);
        // With body y = left, a nose-up pitch is a negative rotation about body y.
        // R = Rz(yaw) * Ry(-pitch) * Rx(roll)
        double[] rz = {cy, -sy, 0, sy, cy, 0, 0, 0, 1};
        double[] ry = {cp, 0, -sp, 0, 1, 0, sp, 0, cp};
        double[] rx = {1, 0, 0, 0, cr, -sr, 0, sr, cr};
        return mul(rz, mul(ry, rx));
    }

    public static double[] mul(double[] a, double[] b) {
        double[] r = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
            }
        }
        return r;
    }

    public static double[] apply(double[] m, double[] v) {
        return new double[]{
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
    }

    public static double[] transpose(double[] m) {
        return new double[]{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
    }

    public static double[] axisAngle(double[] axis, double angle) {
        double n = norm(axis);
        double x = axis[0] / n, y = axis[1] / n, z = axis[2] / n;
        double c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
        return new double[]{
            t * x * x + c, t * x * y - s * z, t * x * z + s * y,
            t * x * y + s * z, t * y * y + c, t * y * z - s * x,
            t * x * z - s * y, t * y * z + s * x, t * z * z + c};
    }

    /** Rotation whose rows are the given orthonormal axes (maps vectors into that basis). */
    public static double[] fromRows(double[] r0, double[] r1, double[] r2) {
        return new double[]{r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2]};
    }

    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    public static double[] cross(double[] a, double[] b) {
        return new double[]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    public static double norm(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    public static double[] normalize(double[] a) {
        double n = norm(a);
        return new double[]{a[0] / n, a[1] / n, a[2] / n};
    }

    public static double[] scale(double[] a, double s) {
        return new double[]{a[0] * s, a[1] * s, a[2] * s};
    }

    public static double[] sub(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    /** Angle between two rotations [rad]. */
    public static double angleBetween(double[] a, double[] b) {
        double[] r = mul(transpose(a), b);
        double tr = (r[0] + r[4] + r[8] - 1) / 2;
        return Math.acos(MathUtil.clamp(tr, -1, 1));
    }
}
