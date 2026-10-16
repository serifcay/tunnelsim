package tunnelsim.map;

/** A position on the road network: an edge and the distance along it [m]. */
public record MapPosition(int edge, double s) {
}
