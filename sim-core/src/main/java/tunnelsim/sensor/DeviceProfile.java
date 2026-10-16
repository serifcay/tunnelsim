package tunnelsim.sensor;

/** Which sensors a phone exposes to a navigation app, and how good its IMU is. */
public enum DeviceProfile {
    /** A modern Android phone: everything in the phone-only sensor set. */
    GENERIC(true, true, true, true, 1.0),
    /** iPhone: no raw GNSS measurements, no ambient light API, no cell info for apps. */
    IOS(false, false, false, true, 1.0),
    /** Budget phone: no barometer and a noisier IMU. */
    LOW_END(true, true, true, false, 2.0);

    public final boolean gnssRaw;
    public final boolean light;
    public final boolean cell;
    public final boolean baro;
    public final double imuQuality;

    DeviceProfile(boolean gnssRaw, boolean light, boolean cell, boolean baro, double imuQuality) {
        this.gnssRaw = gnssRaw;
        this.light = light;
        this.cell = cell;
        this.baro = baro;
        this.imuQuality = imuQuality;
    }
}
