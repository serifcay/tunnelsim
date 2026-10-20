package tunnelsim.map;

import java.util.ArrayList;
import java.util.List;

/**
 * A directed piece of road between two nodes. Edges form a tree rooted at the start of the main
 * road: a fork ends an edge and starts two children (main continuation and ramp). Several main
 * edges share one {@link RoadGeometry}, offset by {@link #geometryOffset}.
 */
public final class Edge {
    public final int id;
    public final String name;
    public final RoadGeometry geometry;
    // Where this edge starts on its geometry (main-road pieces share one geometry), its length, and its parent
    // edge (-1 for the first one).
    public final double geometryOffset;
    public final double length;
    public final int parent;
    /** Distance from the network origin to the start of this edge, along its path [m]. */
    public final double pathStart;
    public final double speedLimit;
    public final boolean ramp;
    final List<Integer> children = new ArrayList<>();
    /** Tunnel intervals in local edge coordinates: {from, to, tunnelId}. */
    final List<double[]> tunnelIntervals = new ArrayList<>();

    Edge(int id, String name, RoadGeometry geometry, double geometryOffset, double length, int parent,
         double pathStart, double speedLimit, boolean ramp) {
        this.id = id;
        this.name = name;
        this.geometry = geometry;
        this.geometryOffset = geometryOffset;
        this.length = length;
        this.parent = parent;
        this.pathStart = pathStart;
        this.speedLimit = speedLimit;
        this.ramp = ramp;
    }

    public List<Integer> children() {
        return children;
    }

    /** True for the last edge of a path (no fork after it). */
    public boolean isLeaf() {
        return children.isEmpty();
    }

    /** Road geometry at distance s along this edge (s is clamped to the edge). */
    public Pose pose(double s) {
        return geometry.pose(geometryOffset + clamp(s));
    }

    public double curvature(double s) {
        return geometry.curvature(geometryOffset + clamp(s));
    }

    public double grade(double s) {
        return geometry.grade(geometryOffset + clamp(s));
    }

    public double elevation(double s) {
        return geometry.elevation(geometryOffset + clamp(s));
    }

    public double x(double s) {
        return geometry.x(geometryOffset + clamp(s));
    }

    public double y(double s) {
        return geometry.y(geometryOffset + clamp(s));
    }

    /** Returns {s, distance} of the point on this edge nearest to (px, py). */
    public double[] nearest(double px, double py) {
        // Search only this edge's part of the geometry, then convert back to edge coordinates.
        double[] r = geometry.nearest(px, py, geometryOffset, geometryOffset + length);
        r[0] -= geometryOffset;
        return r;
    }

    private double clamp(double s) {
        return Math.max(0, Math.min(length, s));
    }

    @Override
    public String toString() {
        return name;
    }
}
