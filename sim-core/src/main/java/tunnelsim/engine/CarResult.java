package tunnelsim.engine;

import tunnelsim.metrics.TrackMetrics;
import tunnelsim.scenario.ScenarioConfig.CarConfig;
import tunnelsim.sensor.SensorLog;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Everything produced for one car: truth, sensor log, every estimator's track and metrics. */
public final class CarResult {
    public final CarConfig car;
    public final TruthTrajectory truth;
    public final SensorLog log;
    // Per estimator id: its recorded track and its summary metrics; plus the event log of this car.
    public final Map<String, Track> tracks = new LinkedHashMap<>();
    public final Map<String, TrackMetrics> metrics = new LinkedHashMap<>();
    public final List<RunEvent> events = new ArrayList<>();
    /** Time ranges with GNSS outages: {start, end}. */
    public final List<double[]> gnssOutages = new ArrayList<>();
    /** Times the car passed each fork, and whether it took the route's branch. */
    public final List<double[]> forkPassages = new ArrayList<>();

    public CarResult(CarConfig car, TruthTrajectory truth, SensorLog log) {
        this.car = car;
        this.truth = truth;
        this.log = log;
    }
}
