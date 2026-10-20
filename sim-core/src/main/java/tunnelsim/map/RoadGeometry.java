package tunnelsim.map;

import tunnelsim.scenario.ScenarioConfig.SegmentConfig;

import java.util.List;

/**
 * A road centreline sampled at a fixed spacing. Heading is integrated from curvature and
 * elevation from grade, so the profiles the estimators match against (curvature, grade,
 * elevation) are exactly consistent with the geometry the car drives.
 */
public final class RoadGeometry {
    public static final double DS = 0.5;

    private final double[] x;
    private final double[] y;
    private final double[] z;
    private final double[] heading;
    private final double[] curvature;
    private final double[] grade;
    private final double length;

    private RoadGeometry(double[] x, double[] y, double[] z, double[] heading, double[] curvature,
                         double[] grade, double length) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.heading = heading;
        this.curvature = curvature;
        this.grade = grade;
        this.length = length;
    }

    /**
     * Builds a centreline from a start pose and a list of segments.
     *
     * @param gradeTransition length of the moving-average window that turns grade steps into
     *                        vertical curves
     */
    public static RoadGeometry build(Pose start, List<SegmentConfig> segments, double gradeTransition) {
        double total = segments.stream().mapToDouble(s -> s.length).sum();
        // One sample every DS (0.5 m) along the whole road.
        int n = (int) Math.round(total / DS) + 1;
        double[] k = new double[n];
        double[] rawGrade = new double[n];

        // Curvature and (step) grade per sample.
        double kPrev = start.curvature();
        double sStart = 0;
        int idx = 0;
        for (SegmentConfig seg : segments) {
            double sEnd = sStart + seg.length;
            double kStart = kPrev;
            // Curvature at the end of this segment: 0 for a straight, constant for an arc, the given end value for
            // a clothoid.
            double kEnd = switch (seg.type) {
                case "straight" -> 0;
                case "arc" -> seg.curvature != null ? seg.curvature : 1.0 / seg.radius;
                case "clothoid" -> seg.curvatureEnd;
                default -> throw new IllegalArgumentException("Unknown segment type " + seg.type);
            };
            // Fill the samples inside this segment. A clothoid's curvature changes linearly from its start value
            // to its end value, which is how real roads ease into and out of curves.
            while (idx < n && idx * DS <= sEnd + 1e-9) {
                double s = idx * DS;
                double f = (s - sStart) / seg.length;
                k[idx] = switch (seg.type) {
                    case "clothoid" -> kStart + (kEnd - kStart) * f;
                    default -> kEnd;
                };
                rawGrade[idx] = seg.grade;
                idx++;
            }
            kPrev = kEnd;
            sStart = sEnd;
        }
        // Rounding can leave a last sample or two: continue the last segment's values.
        for (; idx < n; idx++) {
            k[idx] = kPrev;
            rawGrade[idx] = rawGrade[Math.max(0, idx - 1)];
        }

        // Turn sudden grade changes into gradual vertical curves.
        double[] g = smooth(rawGrade, (int) Math.max(1, Math.round(gradeTransition / DS)), start.grade());

        double[] x = new double[n];
        double[] y = new double[n];
        double[] z = new double[n];
        double[] h = new double[n];
        x[0] = start.x();
        y[0] = start.y();
        z[0] = start.z();
        h[0] = start.heading();
        // Integrate along the road: heading from curvature, x/y from heading, elevation from grade.
        for (int i = 1; i < n; i++) {
            // Midpoint integration of heading and position.
            h[i] = h[i - 1] + 0.5 * (k[i - 1] + k[i]) * DS;
            double hm = 0.5 * (h[i - 1] + h[i]);
            x[i] = x[i - 1] + Math.cos(hm) * DS;
            y[i] = y[i - 1] + Math.sin(hm) * DS;
            z[i] = z[i - 1] + 0.5 * (g[i - 1] + g[i]) * DS;
        }
        return new RoadGeometry(x, y, z, h, k, g, (n - 1) * DS);
    }

    /** Centred moving average; values before the start are padded with the start grade. */
    private static double[] smooth(double[] v, int window, double startValue) {
        int n = v.length;
        double[] out = new double[n];
        int half = window / 2;
        for (int i = 0; i < n; i++) {
            double sum = 0;
            for (int j = i - half; j <= i + half; j++) {
                sum += j < 0 ? startValue : v[Math.min(n - 1, j)];
            }
            out[i] = sum / (2 * half + 1);
        }
        return out;
    }

    public double length() {
        return length;
    }

    /** Everything about the road at distance s, interpolated linearly between the two nearest samples. */
    public Pose pose(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        int j = Math.min(i + 1, x.length - 1);
        double t = f - i;
        return new Pose(
                lerp(x, i, j, t), lerp(y, i, j, t), lerp(z, i, j, t),
                lerp(heading, i, j, t), lerp(curvature, i, j, t), lerp(grade, i, j, t));
    }

    public double curvature(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        return lerp(curvature, i, Math.min(i + 1, x.length - 1), f - i);
    }

    public double grade(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        return lerp(grade, i, Math.min(i + 1, x.length - 1), f - i);
    }

    public double elevation(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        return lerp(z, i, Math.min(i + 1, x.length - 1), f - i);
    }

    public double x(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        return lerp(x, i, Math.min(i + 1, x.length - 1), f - i);
    }

    public double y(double s) {
        double f = clampIndex(s);
        int i = (int) f;
        return lerp(y, i, Math.min(i + 1, x.length - 1), f - i);
    }

    /**
     * Distance along this geometry of the point closest to (px, py), searching [sFrom, sTo].
     * Returns {s, distance}.
     */
    public double[] nearest(double px, double py, double sFrom, double sTo) {
        // First find the closest sample point (brute force over the search range).
        int i0 = (int) Math.max(0, Math.floor(sFrom / DS));
        int i1 = (int) Math.min(x.length - 1, Math.ceil(sTo / DS));
        int best = i0;
        double bestD = Double.MAX_VALUE;
        for (int i = i0; i <= i1; i++) {
            double dx = x[i] - px, dy = y[i] - py;
            double d = dx * dx + dy * dy;
            if (d < bestD) {
                bestD = d;
                best = i;
            }
        }
        // Refine by projecting onto the neighbouring sample segments.
        double bestS = best * DS;
        double bestDist = Math.sqrt(bestD);
        for (int j = Math.max(i0, best - 1); j < Math.min(i1, best + 1); j++) {
            double ax = x[j], ay = y[j], bx = x[j + 1], by = y[j + 1];
            double vx = bx - ax, vy = by - ay;
            double t = Math.max(0, Math.min(1, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)));
            double qx = ax + t * vx, qy = ay + t * vy;
            double d = Math.hypot(px - qx, py - qy);
            if (d < bestDist) {
                bestDist = d;
                bestS = (j + t) * DS;
            }
        }
        return new double[]{bestS, bestDist};
    }

    /** Fractional sample index for distance s, kept inside the road. */
    private double clampIndex(double s) {
        return Math.max(0, Math.min(x.length - 1, s / DS));
    }

    /** Linear interpolation between samples i and j (t from 0 to 1). */
    private static double lerp(double[] a, int i, int j, double t) {
        return a[i] + (a[j] - a[i]) * t;
    }
}
