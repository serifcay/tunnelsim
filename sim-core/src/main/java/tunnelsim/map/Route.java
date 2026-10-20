package tunnelsim.map;

import java.util.List;

/** The route the navigation app guides along: one leaf of the road tree. */
public final class Route {
    private final RoadNetwork network;
    private final int leaf;
    /** Path offset of the fork where the route leaves the main road, or NaN if it never does. */
    private final double maneuverOffset;

    public Route(RoadNetwork network, List<Double> takeBranchAt) {
        this.network = network;
        int e = 0;
        double maneuver = Double.NaN;
        // Walk from the start: at each fork, take the ramp if the scenario says the route takes this branch. The
        // first ramp taken is the maneuver the callouts are for.
        while (!network.edge(e).isLeaf()) {
            Edge edge = network.edge(e);
            double forkAt = edge.pathStart + edge.length;
            boolean take = takeBranchAt.stream().anyMatch(b -> Math.abs(b - forkAt) < 1.0);
            if (take && Double.isNaN(maneuver)) {
                maneuver = forkAt;
            }
            e = take ? edge.children().get(1) : edge.children().get(0);
        }
        this.leaf = e;
        this.maneuverOffset = maneuver;
    }

    public int leaf() {
        return leaf;
    }

    /** True if the route takes an exit (there is something to announce). */
    public boolean hasManeuver() {
        return !Double.isNaN(maneuverOffset);
    }

    public double maneuverOffset() {
        return maneuverOffset;
    }

    /** True if the edge lies on the planned route. */
    public boolean contains(int edge) {
        return network.isAncestorOrSelf(edge, leaf);
    }

    /** True if a position on {@code edge} has left the route at a fork before the maneuver. */
    public boolean leftBeforeManeuver(int edge) {
        return hasManeuver() && !contains(edge) && network.forkBetween(edge, leaf) < maneuverOffset - 1;
    }

    /** The child the route continues on when reaching the end of {@code edge}. */
    public int next(int edge) {
        for (int child : network.edge(edge).children()) {
            if (network.isAncestorOrSelf(child, leaf)) {
                return child;
            }
        }
        return network.edge(edge).children().get(0);
    }
}
