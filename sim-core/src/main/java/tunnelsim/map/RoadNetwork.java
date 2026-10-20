package tunnelsim.map;

import tunnelsim.map.Features.Beacon;
import tunnelsim.map.Features.CellZone;
import tunnelsim.map.Features.Portal;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioConfig.BranchConfig;
import tunnelsim.scenario.ScenarioConfig.RoadConfig;
import tunnelsim.scenario.ScenarioConfig.TrafficZoneConfig;
import tunnelsim.scenario.ScenarioConfig.TunnelConfig;
import tunnelsim.scenario.SensorsConfig;
import tunnelsim.util.Rng;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * The road network: a tree of edges rooted at the start of the main road, plus tunnels and
 * point features. Every point has a <em>path offset</em>, its distance from the origin along the
 * unique path that reaches it; offsets on different branches overlap, so a path offset is only
 * meaningful together with the path (identified by its leaf edge).
 */
public final class RoadNetwork {
    private final List<Edge> edges = new ArrayList<>();
    private final List<TunnelConfig> tunnels = new ArrayList<>();
    private final List<Portal> portals = new ArrayList<>();
    private final List<CellZone> cells = new ArrayList<>();
    private final List<Beacon> beacons = new ArrayList<>();
    private final List<TrafficZoneConfig> traffic;
    private final List<Double> forks = new ArrayList<>();
    private final boolean night;

    private RoadNetwork(RoadConfig road) {
        this.traffic = road.traffic;
        this.night = "night".equalsIgnoreCase(road.lighting);
    }

    /** Builds the network from the scenario: the main road, one ramp per fork, tunnels, cell zones and beacons. */
    public static RoadNetwork build(ScenarioConfig cfg, Rng rng) {
        RoadConfig road = cfg.road;
        RoadNetwork net = new RoadNetwork(road);
        double startGrade = road.main.get(0).grade;
        // The main road starts at the origin heading east.
        RoadGeometry main = RoadGeometry.build(new Pose(0, 0, 0, 0, 0, startGrade), road.main, road.gradeTransition);

        List<BranchConfig> branches = new ArrayList<>(road.branches);
        branches.sort(Comparator.comparingDouble(b -> b.at));

        // Main road pieces between forks.
        List<Double> cuts = new ArrayList<>();
        cuts.add(0.0);
        for (BranchConfig b : branches) {
            cuts.add(b.at);
            net.forks.add(b.at);
        }
        cuts.add(main.length());
        // Each piece becomes an edge; piece i continues from piece i-1 (its parent in the tree).
        for (int i = 0; i + 1 < cuts.size(); i++) {
            double from = cuts.get(i), to = cuts.get(i + 1);
            int parent = i == 0 ? -1 : i - 1;
            net.edges.add(new Edge(i, cuts.size() > 2 ? "main-" + i : "main", main, from, to - from, parent, from,
                    road.speedLimit, false));
            if (parent >= 0) {
                net.edges.get(parent).children.add(i);
            }
        }
        // Ramps.
        for (int j = 0; j < branches.size(); j++) {
            BranchConfig b = branches.get(j);
            int parent = j; // main piece j ends at this fork
            // A ramp starts at the fork with the main road's position, heading, curvature and grade there.
            Pose p = main.pose(b.at);
            RoadGeometry g = RoadGeometry.build(p, b.ramp, road.gradeTransition);
            int id = net.edges.size();
            net.edges.add(new Edge(id, "ramp@" + Math.round(b.at), g, 0, g.length(), parent, b.at, b.speedLimit, true));
            net.edges.get(parent).children.add(id);
        }

        // Tunnels.
        for (int t = 0; t < road.tunnels.size(); t++) {
            TunnelConfig tc = road.tunnels.get(t);
            net.tunnels.add(tc);
            for (Edge e : net.edges) {
                if (e.ramp) {
                    continue;
                }
                // Part of the tunnel that lies on this main-road edge, in the edge's own coordinates.
                double from = Math.max(tc.from, e.pathStart) - e.pathStart;
                double to = Math.min(tc.to, e.pathStart + e.length) - e.pathStart;
                if (to > from) {
                    e.tunnelIntervals.add(new double[]{from, to, t});
                }
            }
            // The tunnel's mouths: entry and exit on the main road.
            net.addPortal(t, net.positionOnMain(tc.from), true);
            net.addPortal(t, net.positionOnMain(tc.to), false);
            for (int j = 0; j < branches.size(); j++) {
                BranchConfig b = branches.get(j);
                // A fork inside the tunnel: the ramp stays in the tunnel for its first tunnelUntil metres and has
                // its own exit portal.
                if (b.at > tc.from && b.at < tc.to && b.tunnelUntil > 0) {
                    Edge ramp = net.edges.get(net.forks.size() + 1 + j);
                    double until = Math.min(b.tunnelUntil, ramp.length);
                    ramp.tunnelIntervals.add(new double[]{0, until, t});
                    net.addPortal(t, new MapPosition(ramp.id, until), false);
                }
            }
        }

        net.placeCells(cfg.sensors.cell);
        net.placeBeacons(cfg.sensors.ble, rng);
        return net;
    }

    private void addPortal(int tunnel, MapPosition pos, boolean entry) {
        Edge e = edges.get(pos.edge());
        portals.add(new Portal(tunnel, pos, entry, e.x(pos.s()), e.y(pos.s())));
    }

    private void placeCells(SensorsConfig.CellConfig cc) {
        int id = 1;
        for (TunnelConfig t : tunnels) {
            List<Double> at = new ArrayList<>();
            // In-tunnel coverage: a handover just inside the entry, one every handoverSpacing metres, and one just
            // after the exit.
            at.add(t.from + 50);
            for (double s = t.from + cc.handoverSpacing; s < t.to - cc.handoverSpacing / 3; s += cc.handoverSpacing) {
                at.add(s);
            }
            at.add(Math.min(mainLength(), t.to + 50));
            for (double s : at) {
                MapPosition p = positionOnMain(s);
                Edge e = edges.get(p.edge());
                cells.add(new CellZone(id++, p, e.x(p.s()), e.y(p.s())));
            }
        }
    }

    private void placeBeacons(SensorsConfig.BleConfig bc, Rng rng) {
        int id = 1;
        for (Edge e : edges) {
            for (double[] iv : e.tunnelIntervals) {
                // Beacons at regular spacing through every tunnel section, half a spacing in from its start.
                for (double s = iv[0] + bc.spacing / 2; s < iv[1]; s += bc.spacing) {
                    Pose p = e.pose(s);
                    // Mounted on the right-hand wall.
                    double nx = Math.sin(p.heading()), ny = -Math.cos(p.heading());
                    double x = p.x() + bc.lateralOffset * nx;
                    double y = p.y() + bc.lateralOffset * ny;
                    beacons.add(new Beacon(id++, x, y, p.z() + bc.height,
                            // The surveyed position differs a little from the true one; some beacons are dead.
                            x + rng.gaussian(bc.positionError), y + rng.gaussian(bc.positionError),
                            rng.chance(bc.failedFraction)));
                }
            }
        }
    }

    // ---- Topology ----

    public List<Edge> edges() {
        return edges;
    }

    public Edge edge(int id) {
        return edges.get(id);
    }

    public List<Double> forks() {
        return forks;
    }

    /** Total length of the main road (follow the first child at every fork to its end). */
    public double mainLength() {
        Edge e = edges.get(0);
        while (!e.isLeaf()) {
            e = edges.get(e.children.get(0));
        }
        return e.pathStart + e.length;
    }

    /** The leaf edge of the main road. */
    public int mainLeaf() {
        Edge e = edges.get(0);
        while (!e.isLeaf()) {
            e = edges.get(e.children.get(0));
        }
        return e.id;
    }

    /** All leaf edges, i.e. every possible destination (main road end and each ramp). */
    public List<Integer> leaves() {
        return edges.stream().filter(Edge::isLeaf).map(e -> e.id).toList();
    }

    /** True if edge a lies on the path from the origin to edge b (a may equal b). */
    public boolean isAncestorOrSelf(int a, int b) {
        int e = b;
        while (e >= 0) {
            if (e == a) {
                return true;
            }
            // Walk from b back towards the start; if we meet a on the way, a is on b's path.
            e = edges.get(e).parent;
        }
        return false;
    }

    /** True if both edges lie on one path from the origin. */
    public boolean onCommonPath(int a, int b) {
        return isAncestorOrSelf(a, b) || isAncestorOrSelf(b, a);
    }

    /** Distance from the start of the network to this position, along the path that reaches it. */
    public double pathOffset(MapPosition p) {
        return edges.get(p.edge()).pathStart + p.s();
    }

    /** The position at a path offset along the path ending in {@code leaf} (clamped to the path). */
    public MapPosition positionOnPath(int leaf, double offset) {
        int e = leaf;
        while (e >= 0) {
            Edge edge = edges.get(e);
            // Walk back from the leaf towards the start until the edge that contains the offset.
            if (offset >= edge.pathStart || edge.parent < 0) {
                return new MapPosition(e, Math.max(0, Math.min(edge.length, offset - edge.pathStart)));
            }
            e = edge.parent;
        }
        throw new IllegalStateException();
    }

    /** Position at a distance along the main road. */
    public MapPosition positionOnMain(double offset) {
        return positionOnPath(mainLeaf(), offset);
    }

    /** A leaf edge reachable from {@code edge}, preferring the main continuation at forks. */
    public int defaultLeaf(int edge) {
        Edge e = edges.get(edge);
        while (!e.isLeaf()) {
            e = edges.get(e.children.get(0));
        }
        return e.id;
    }

    /** Path offset of the end of the path ending in {@code leaf}. */
    public double pathEnd(int leaf) {
        Edge e = edges.get(leaf);
        return e.pathStart + e.length;
    }

    /**
     * Signed along-road error of {@code estimate} relative to {@code truth} [m]. When the two lie
     * on different branches, the distance is measured through the fork and flagged.
     */
    public PathError error(MapPosition estimate, MapPosition truth) {
        double oe = pathOffset(estimate), ot = pathOffset(truth);
        // Same path: simply the difference of the two distances from the start.
        if (onCommonPath(estimate.edge(), truth.edge())) {
            return new PathError(oe - ot, false);
        }
        // Different branches: back from the estimate to the fork, then forward to the car.
        double fork = forkBetween(estimate.edge(), truth.edge());
        return new PathError((oe - fork) + (ot - fork), true);
    }

    /** Path offset where the paths to two edges split. */
    public double forkBetween(int a, int b) {
        int e = a;
        // Walk up from a until reaching an edge that also leads to b; its end is the fork.
        while (e >= 0 && !isAncestorOrSelf(e, b)) {
            e = edges.get(e).parent;
        }
        Edge common = edges.get(Math.max(0, e));
        return common.pathStart + common.length;
    }

    /** Along-road error and whether the estimate is on a different branch than the car. */
    public record PathError(double meters, boolean wrongBranch) {
    }

    // ---- Geometry lookups ----

    /** Road geometry at a map position. */
    public Pose pose(MapPosition p) {
        return edges.get(p.edge()).pose(p.s());
    }

    /** Nearest point on any edge to (x, y): {edge, s, distance}. */
    public double[] nearest(double x, double y) {
        double[] best = null;
        for (Edge e : edges) {
            double[] r = e.nearest(x, y);
            if (best == null || r[1] < best[2]) {
                best = new double[]{e.id, r[0], r[1]};
            }
        }
        return best;
    }

    /** Nearest point to (x, y) on the path ending in {@code leaf}: {edge, s, distance}. */
    public double[] nearestOnPath(int leaf, double x, double y) {
        double[] best = null;
        for (int e = leaf; e >= 0; e = edges.get(e).parent) {
            double[] r = edges.get(e).nearest(x, y);
            if (best == null || r[1] < best[2]) {
                best = new double[]{e, r[0], r[1]};
            }
        }
        return best;
    }

    // ---- Tunnels and features ----

    public List<TunnelConfig> tunnels() {
        return tunnels;
    }

    public List<Portal> portals() {
        return portals;
    }

    public List<CellZone> cells() {
        return cells;
    }

    public List<Beacon> beacons() {
        return beacons;
    }

    public boolean night() {
        return night;
    }

    /** Index of the tunnel containing the position, or -1. */
    public int tunnelAt(MapPosition p) {
        return tunnelAt(p.edge(), p.s());
    }

    /** Index of the tunnel containing (edge, s), or -1. */
    public int tunnelAt(int edge, double s) {
        for (double[] iv : edges.get(edge).tunnelIntervals) {
            if (s >= iv[0] && s <= iv[1]) {
                return (int) iv[2];
            }
        }
        return -1;
    }

    /** Distance from the tunnel entry along the road, for positions inside a tunnel [m]; else -1. */
    public double distanceIntoTunnel(MapPosition p) {
        int t = tunnelAt(p);
        if (t < 0) {
            return -1;
        }
        return pathOffset(p) - tunnels.get(t).from;
    }

    /** Tunnel intervals on an edge, as {from, to, tunnelId} in local coordinates. */
    public List<double[]> tunnelIntervals(int edge) {
        return edges.get(edge).tunnelIntervals;
    }

    /**
     * GNSS signal quality in [0, 1]: 0 inside tunnels, ramping up through the degraded zone
     * around each portal, 1 in open sky.
     */
    public double gnssQuality(MapPosition p) {
        if (tunnelAt(p) >= 0) {
            return 0;
        }
        Pose pose = pose(p);
        // Within the degraded zone around any portal, quality falls linearly from 1 to 0.2 at the mouth.
        double q = 1;
        for (Portal portal : portals) {
            double zone = tunnels.get(portal.tunnel()).gnssDegradedZone;
            double d = Math.hypot(pose.x() - portal.x(), pose.y() - portal.y());
            if (d < zone) {
                q = Math.min(q, 0.2 + 0.8 * d / zone);
            }
        }
        return q;
    }

    /** Speed the navigation app expects at a position: live traffic if reported, else the limit. */
    public double expectedSpeed(MapPosition p) {
        Edge e = edges.get(p.edge());
        // Reported traffic zones are defined along the main road.
        if (!e.ramp) {
            double off = pathOffset(p);
            for (TrafficZoneConfig z : traffic) {
                if (off >= z.from && off <= z.to) {
                    return z.speed;
                }
            }
        }
        return e.speedLimit;
    }
}
