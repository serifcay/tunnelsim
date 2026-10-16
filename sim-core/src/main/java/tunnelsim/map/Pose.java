package tunnelsim.map;

/**
 * A point on a road centreline. Heading is measured counter-clockwise from east [rad],
 * curvature is positive for left turns [1/m], grade is rise over run.
 */
public record Pose(double x, double y, double z, double heading, double curvature, double grade) {
}
