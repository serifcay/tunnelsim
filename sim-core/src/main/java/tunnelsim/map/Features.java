package tunnelsim.map;

/** Point features placed on the road network. */
public final class Features {
    private Features() {
    }

    /** A tunnel mouth. {@code entry} is true where cars driving forward enter the tunnel. */
    public record Portal(int tunnel, MapPosition position, boolean entry, double x, double y) {
    }

    /** A place where the phone hands over between serving cells. */
    public record CellZone(int cellId, MapPosition position, double x, double y) {
    }

    /**
     * A BLE beacon. (x, y, z) is where it really is; (mx, my) is where the survey says it is,
     * which is what the estimators use.
     */
    public record Beacon(int id, double x, double y, double z, double mx, double my, boolean failed) {
    }
}
