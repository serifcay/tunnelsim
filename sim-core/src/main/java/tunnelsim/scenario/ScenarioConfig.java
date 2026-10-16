package tunnelsim.scenario;

import java.util.ArrayList;
import java.util.List;

/**
 * Root of a scenario file. All config classes use public mutable fields with defaults so that
 * YAML files only need to state what differs, and the UI can tweak values before a re-run.
 * Units are SI (m, s, m/s, rad) unless a field name says otherwise (e.g. {@code ...Deg}).
 */
public class ScenarioConfig {
    public String name = "unnamed";
    public String description = "";
    public long seed = 42;
    /** Simulated time [s]; the run also ends when every car reaches the end of its road. */
    public double duration = 600;
    /** GENERIC, IOS or LOW_END. */
    public String device = "GENERIC";
    public OptionsConfig options = new OptionsConfig();
    public RoadConfig road = new RoadConfig();
    public RouteConfig route = new RouteConfig();
    public List<CarConfig> cars = new ArrayList<>(List.of(new CarConfig()));
    public SensorsConfig sensors = new SensorsConfig();
    public List<String> estimators = new ArrayList<>(List.of("A", "B", "C", "D"));
    public EstimatorConfig estimator = new EstimatorConfig();
    public NavConfig nav = new NavConfig();

    public static class OptionsConfig {
        /** Option E: BLE beacons installed in tunnels. */
        public boolean ble = false;
        /** Option E: vehicle speed from OBD-II / Android Automotive. */
        public boolean obd = false;
    }

    public static class RoadConfig {
        /** Default speed limit on the main road [m/s] (22.2 = 80 km/h). */
        public double speedLimit = 22.2;
        /** Length over which grade changes are smoothed (vertical curves) [m]. */
        public double gradeTransition = 80;
        /** "day" or "night"; drives the ambient light model. */
        public String lighting = "day";
        public List<SegmentConfig> main = new ArrayList<>();
        public List<TunnelConfig> tunnels = new ArrayList<>();
        public List<BranchConfig> branches = new ArrayList<>();
        /** Live traffic speeds the navigation app believes in (used as a speed prior). */
        public List<TrafficZoneConfig> traffic = new ArrayList<>();
    }

    public static class SegmentConfig {
        /** straight, arc or clothoid. */
        public String type = "straight";
        public double length = 100;
        /** Arc curvature [1/m], positive turns left. Alternative to radius. */
        public Double curvature;
        /** Arc radius [m], positive turns left. */
        public Double radius;
        /** Curvature at the end of a clothoid [1/m]; it starts at the previous segment's value. */
        public double curvatureEnd = 0;
        /** Road grade (rise over run), e.g. -0.04 is 4% downhill. */
        public double grade = 0;
    }

    public static class TunnelConfig {
        /** Tunnel extent along the main road [m]. */
        public double from;
        public double to;
        /** Distance outside each portal where GNSS is degraded [m]. */
        public double gnssDegradedZone = 60;
        /** Spacing of ceiling lights [m]; 0 disables the periodic light pattern. */
        public double lightSpacing = 12;
    }

    public static class BranchConfig {
        /** Position of the fork along the main road [m]. */
        public double at;
        public double speedLimit = 16.7;
        public List<SegmentConfig> ramp = new ArrayList<>();
        /** Length of the ramp that is still inside the tunnel [m] (if the fork is in a tunnel). */
        public double tunnelUntil = 0;
    }

    public static class TrafficZoneConfig {
        /** Zone along the main road [m]. */
        public double from;
        public double to;
        /** Reported traffic speed [m/s]. */
        public double speed;
    }

    public static class RouteConfig {
        /** Fork positions (main-road metres) where the planned route takes the ramp. */
        public List<Double> takeBranchAt = new ArrayList<>();
    }

    public static class CarConfig {
        public String id = "car1";
        /** Seconds after the simulation start when the car starts driving. */
        public double startTime = 5;
        public double startSpeed = 0;
        public DriverConfig driver = new DriverConfig();
        public MountConfig mount = new MountConfig();
        public List<EventConfig> events = new ArrayList<>();
    }

    public static class DriverConfig {
        /** Multiplier on the speed limit the driver aims for. */
        public double speedFactor = 1.0;
        /** Probability of following the planned route at a fork. */
        public double pFollowRoute = 1.0;
        public double maxAccel = 2.0;
        public double comfortDecel = 2.5;
        /** Lateral acceleration the driver accepts in curves [m/s^2]. */
        public double maxLateralAccel = 2.0;
        /** Std-dev of the driver's slowly varying speed fluctuation [m/s]. */
        public double speedJitter = 0.7;
    }

    public static class MountConfig {
        /**
         * Phone orientation relative to the nominal portrait dashboard mount, drawn uniformly
         * from the given [min, max] ranges in degrees.
         */
        public double[] yawDeg = {-15, 15};
        public double[] pitchDeg = {-25, -5};
        public double[] rollDeg = {-5, 5};
    }

    public static class EventConfig {
        /** Distance along the car's path where the event starts [m]. */
        public double at;
        /** slowTo, stop, stopAndGo, hardBrake, mountShift. */
        public String type;
        public double speed = 0;
        public double duration = 0;
        public double length = 0;
        public double stopTime = 10;
        public double goTime = 15;
        public double decel = 6;
        public double yawDeg = 0;
        public double pitchDeg = 0;
        public double rollDeg = 0;
    }

    public static class NavConfig {
        /** Distances before the maneuver at which callouts fire [m]. */
        public List<Double> calloutDistances = new ArrayList<>(List.of(1000.0, 400.0, 100.0));
        /** MEAN or CONSERVATIVE. */
        public String calloutPolicy = "CONSERVATIVE";
        /** CONSERVATIVE fires once P(remaining <= D) reaches this value. */
        public double calloutAlpha = 0.2;
        /** A callout is late if the true remaining distance is below D minus this [m]. */
        public double lateTolerance = 50;
        public double rerouteThreshold = 0.9;
        /** Display smoother time constant [s]. */
        public double smoothingTau = 2.0;
    }
}
