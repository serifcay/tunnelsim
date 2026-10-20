package tunnelsim.nav;

import tunnelsim.estimator.Estimate;
import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;

/**
 * What the driver sees: the marker moves with the estimated speed and blends towards the
 * estimate over a time constant. It never moves backwards along the road; it only jumps when the
 * estimate changes branch or is too far ahead to blend.
 */
public final class DisplaySmoother {
    private static final double SNAP_DISTANCE = 200;

    private final RoadNetwork net;
    private final double tau;
    private int leaf = -1;
    private double offset;

    public DisplaySmoother(RoadNetwork net, double tau) {
        this.net = net;
        this.tau = tau;
    }

    /** Moves the displayed marker one display step (dt) towards the latest estimate. */
    public void update(Estimate e, double dt) {
        if (!e.valid()) {
            return;
        }
        // Jump straight to the estimate on the first call, when it changes branch, or when it is too far away to
        // blend.
        if (leaf < 0 || !net.onCommonPath(leaf, e.leaf()) || Math.abs(e.offset() - offset) > SNAP_DISTANCE) {
            leaf = e.leaf();
            offset = e.offset();
            return;
        }
        leaf = e.leaf();
        // Advance at the estimated speed, then close a share dt/tau of the remaining gap to the estimate. Never
        // move backwards, never past the end of the road.
        double predicted = offset + e.speed() * dt;
        double next = predicted + (e.offset() - predicted) * Math.min(1, dt / tau);
        offset = Math.min(net.pathEnd(leaf), Math.max(offset, next));
    }

    public boolean valid() {
        return leaf >= 0;
    }

    public int leaf() {
        return leaf;
    }

    public double offset() {
        return offset;
    }

    public MapPosition position() {
        return net.positionOnPath(leaf, offset);
    }
}
