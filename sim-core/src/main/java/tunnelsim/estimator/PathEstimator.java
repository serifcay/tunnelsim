package tunnelsim.estimator;

import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.util.MathUtil;

/**
 * Base for estimators that keep a single hypothesis: a path (leaf edge) and a Gaussian belief
 * of the offset along it. They follow the planned route through forks and only change path when
 * GNSS fixes clearly put the car on another branch.
 */
abstract class PathEstimator implements Estimator {
    // A fix more than this far from our path [m] means the car is probably on another branch.
    private static final double SWITCH_DISTANCE = 30;

    protected final EstimatorContext ctx;
    protected final RoadNetwork net;
    protected final GnssTracker gnss = new GnssTracker();
    protected int leaf;
    protected boolean initialised;
    protected double t;

    PathEstimator(EstimatorContext ctx) {
        this.ctx = ctx;
        this.net = ctx.net();
        this.leaf = ctx.route().leaf();
    }

    /**
     * Matches a fix to the road. Switches path if the fix is clearly on another branch.
     * Returns {offset on the (possibly new) path, distance from the road}, or null if far from any road.
     */
    protected double[] matchFix(GnssFix fix) {
        // Normal case: the fix lies close to the path we are on; return its offset along it.
        double[] onPath = net.nearestOnPath(leaf, fix.x(), fix.y());
        if (onPath[2] < SWITCH_DISTANCE) {
            return new double[]{net.edge((int) onPath[0]).pathStart + onPath[1], onPath[2]};
        }
        // Otherwise look at the whole network: if the fix is near another road, switch to that path.
        double[] any = net.nearest(fix.x(), fix.y());
        if (any[2] > 2 * SWITCH_DISTANCE) {
            return null;
        }
        int edge = (int) any[0];
        leaf = net.defaultLeaf(edge);
        onPathSwitched();
        return new double[]{net.edge(edge).pathStart + any[1], any[2]};
    }

    /** Hook for subclasses when a GNSS fix moved the estimate onto another branch. */
    protected void onPathSwitched() {
    }

    /** Edge and position on that edge for a path offset on the current path. */
    protected MapPosition position(double offset) {
        return net.positionOnPath(leaf, offset);
    }

    /** Keeps an offset between the start and the end of the current path. */
    protected double clampOffset(double offset) {
        return MathUtil.clamp(offset, 0, net.pathEnd(leaf));
    }

    /** Single hypothesis: either fully on the planned route (0) or fully off it (1). */
    protected double pOffRoute() {
        return ctx.route().contains(leaf) ? 0 : 1;
    }

    /** Packs offset, uncertainty and speed into an Estimate (invalid before the first GNSS fix). */
    protected Estimate build(double offset, double sigma, double speed, double sigmaSpeed) {
        if (!initialised) {
            return Estimate.invalid(t);
        }
        MapPosition p = position(offset);
        return new Estimate(t, true, p.edge(), p.s(), leaf, offset, sigma, speed, sigmaSpeed, pOffRoute(),
                gnss.mode(t), null);
    }

    /** Gaussian probability that the remaining distance to a route point is at most d. */
    protected double gaussRemaining(double offset, double sigma, double targetOffset, double d) {
        if (!initialised) {
            return 0;
        }
        if (!ctx.route().contains(leaf)) {
            // Off the route: the maneuver was passed only if the estimate left the route at its fork.
            return net.forkBetween(leaf, ctx.route().leaf()) >= targetOffset - 1 ? 1 : 0;
        }
        // Normal distribution: chance that the remaining distance is at most d, given mean and sigma.
        double remaining = targetOffset - offset;
        return MathUtil.normalCdf((d - remaining) / Math.max(sigma, 0.1));
    }

    // A single path is either consistent with the branch (1) or not (0).
    @Override
    public double probOnSubtree(int edge) {
        return net.isAncestorOrSelf(edge, leaf) || net.isAncestorOrSelf(leaf, edge) ? 1 : 0;
    }

    @Override
    public double probOnPath(int otherLeaf) {
        return net.isAncestorOrSelf(leaf, otherLeaf) || net.isAncestorOrSelf(otherLeaf, leaf) ? 1 : 0;
    }
}
