package tunnelsim.engine;

import tunnelsim.align.Aligner;
import tunnelsim.align.VehicleImu;
import tunnelsim.estimator.Estimate;
import tunnelsim.estimator.Estimator;
import tunnelsim.estimator.EstimatorFactory;
import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.metrics.TrackMetrics;
import tunnelsim.nav.CalloutTracker;
import tunnelsim.nav.DisplaySmoother;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioConfig.CarConfig;
import tunnelsim.sensor.SensorEvent;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.sensor.SensorEvent.ImuSample;
import tunnelsim.sensor.SensorLog;
import tunnelsim.sensor.SensorSuite;
import tunnelsim.util.Rng;
import tunnelsim.vehicle.Driver;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs a scenario: for each car, generate the truth and the phone's sensor log, then replay the
 * log through the alignment front-end, every estimator and the navigation layer, recording at
 * 10 Hz. Fully deterministic for a given scenario (including its seed).
 */
public final class SimulationRun {
    private static final double OUTAGE = 3.0;

    private SimulationRun() {
    }

    public static RunResult run(ScenarioConfig cfg) {
        return run(cfg, false);
    }

    /** @param batch skip data only the UI needs (particle snapshots) */
    public static RunResult run(ScenarioConfig cfg, boolean batch) {
        long start = System.nanoTime();
        // Build the road network and the planned route once; every car drives on them.
        RoadNetwork net = RoadNetwork.build(cfg, Rng.derive(cfg.seed, "map"));
        Route route = new Route(net, cfg.route.takeBranchAt);
        List<CarResult> cars = new ArrayList<>();
        for (CarConfig car : cfg.cars) {
            // For each car: simulate the true drive, record what its phone measures, then run every estimator on
            // that recording.
            long carSeed = cfg.seed * 1_000_003L + car.id.hashCode();
            TruthTrajectory truth = new Driver(net, route, car, Rng.derive(cfg.seed, "car/" + car.id)).simulate(cfg.duration);
            SensorLog log = SensorSuite.record(truth, net, cfg, carSeed);
            cars.add(runCar(cfg, net, route, car, truth, log, carSeed, batch));
        }
        return new RunResult(cfg, net, route, cars, (System.nanoTime() - start) / 1e9);
    }

    /** Replays one car's sensor log through the alignment, every estimator and the navigation layer. */
    private static CarResult runCar(ScenarioConfig cfg, RoadNetwork net, Route route, CarConfig car,
                                    TruthTrajectory truth, SensorLog log, long seed, boolean batch) {
        CarResult result = new CarResult(car, truth, log);
        truthEvents(result, net, route);

        // One shared alignment front-end: all estimators receive the same car-frame IMU data.
        Aligner aligner = new Aligner("TRUTH".equalsIgnoreCase(cfg.estimator.alignment) ? truth::mountAt : null,
                cfg.estimator.accelScaleCalibration);
        List<Estimator> estimators = new ArrayList<>();
        List<Track> tracks = new ArrayList<>();
        List<DisplaySmoother> displays = new ArrayList<>();
        List<CalloutTracker> callouts = new ArrayList<>();
        List<boolean[]> rerouted = new ArrayList<>();
        // Per estimator: the estimator itself, its recorded track, its display smoother and callout tracker.
        for (String id : cfg.estimators) {
            Estimator est = EstimatorFactory.create(id, net, route, cfg, seed);
            if (batch && est instanceof tunnelsim.estimator.RoadParticleFilter pf) {
                pf.setSnapshots(false);
            }
            estimators.add(est);
            Track track = new Track(id);
            tracks.add(track);
            result.tracks.put(id, track);
            displays.add(new DisplaySmoother(net, cfg.nav.smoothingTau));
            callouts.add(new CalloutTracker(route, cfg.nav));
            rerouted.add(new boolean[1]);
        }

        // Slope source for re-aligning the phone at stops after handling: the most capable estimator.
        for (String preferred : List.of("D+E", "D", "C")) {
            int k = cfg.estimators.indexOf(preferred);
            if (k >= 0) {
                aligner.setGradeProvider(estimators.get(k)::gradeAtEstimate, cfg.estimator.slopeAwareRealignment);
                break;
            }
        }
        if (!cfg.estimators.contains("D+E") && !cfg.estimators.contains("D") && !cfg.estimators.contains("C")) {
            aligner.setGradeProvider(null, cfg.estimator.slopeAwareRealignment);
        }

        double nextRecord = 0;
        double end = truth.duration();
        // Feed every sensor event in time order. Before each event, record the state at every 0.1 s mark that has
        // passed.
        for (SensorEvent e : log.events()) {
            while (e.t() >= nextRecord && nextRecord <= end) {
                record(nextRecord, result, net, route, cfg, estimators, tracks, displays, callouts, rerouted);
                nextRecord += RunConstants.RECORD_DT;
            }
            // IMU samples go through the aligner first; the estimators get both the raw and the car-frame sample.
            // CPU time per estimator is measured as we go.
            if (e instanceof ImuSample imu) {
                VehicleImu vi = aligner.onImu(imu);
                for (int k = 0; k < estimators.size(); k++) {
                    long t0 = System.nanoTime();
                    estimators.get(k).onSensor(e);
                    estimators.get(k).onVehicleImu(vi);
                    tracks.get(k).nanos += System.nanoTime() - t0;
                }
            } else {
                if (e instanceof GnssFix fix) {
                    aligner.onGnss(fix);
                }
                for (int k = 0; k < estimators.size(); k++) {
                    long t0 = System.nanoTime();
                    estimators.get(k).onSensor(e);
                    tracks.get(k).nanos += System.nanoTime() - t0;
                }
            }
        }
        // Record the remaining time after the last event.
        while (nextRecord <= end) {
            record(nextRecord, result, net, route, cfg, estimators, tracks, displays, callouts, rerouted);
            nextRecord += RunConstants.RECORD_DT;
        }
        for (int k = 0; k < estimators.size(); k++) {
            Track track = tracks.get(k);
            if (estimators.get(k) instanceof tunnelsim.estimator.RoadParticleFilter pf) {
                track.reinitialisations = pf.reinitialisations();
            }
            result.events.addAll(track.events);
            // Summarise each track into metrics once the run is over.
            result.metrics.put(track.estimatorId, TrackMetrics.compute(track, result, net, route));
        }
        result.events.sort((a, b) -> Double.compare(a.t(), b.t()));
        return result;
    }

    /**
     * Takes one 10 Hz sample of every estimator: estimate, error against the truth, display marker, callouts,
     * reroute.
     */
    private static void record(double t, CarResult result, RoadNetwork net, Route route, ScenarioConfig cfg,
                               List<Estimator> estimators, List<Track> tracks, List<DisplaySmoother> displays,
                               List<CalloutTracker> callouts, List<boolean[]> rerouted) {
        TruthTrajectory truth = result.truth;
        int i = truth.indexAt(t);
        MapPosition truthPos = truth.position(i);
        for (int k = 0; k < estimators.size(); k++) {
            Estimator est = estimators.get(k);
            Track track = tracks.get(k);
            long t0 = System.nanoTime();
            Estimate e = est.estimate();
            double pTruth = est.probOnPath(truth.leaf);
            // Probability of the branch the car took at the last fork it passed (for the branch-decision metric).
            double pBranch = 1;
            for (double[] fp : result.forkPassages) {
                if (fp[0] <= t) {
                    pBranch = est.probOnSubtree((int) fp[2]);
                }
            }
            track.nanos += System.nanoTime() - t0;

            DisplaySmoother display = displays.get(k);
            // What the driver would see: the smoothed marker.
            display.update(e, RunConstants.RECORD_DT);

            double err = Double.NaN, dispErr = Double.NaN;
            boolean wrong = false, dispWrong = false;
            if (e.valid()) {
                // Along-road error of the estimate and of the displayed marker against the true position.
                RoadNetwork.PathError pe = net.error(new MapPosition(e.edge(), e.s()), truthPos);
                err = pe.meters();
                wrong = pe.wrongBranch();
            }
            if (display.valid()) {
                RoadNetwork.PathError pe = net.error(display.position(), truthPos);
                dispErr = pe.meters();
                dispWrong = pe.wrongBranch();
            }
            track.samples.add(new Track.Sample(t, e, err, wrong, pTruth, pBranch, display.valid() ? display.leaf() : -1,
                    display.offset(), dispErr, dispWrong));

            if (e.valid()) {
                for (CalloutTracker.Callout c : callouts.get(k).update(t, est, truth.edge(i), truth.offset(i), truth.v(i))) {
                    track.callouts.add(c);
                    String text = c.fired()
                            ? String.format("callout %.0f m (true %.0f m, %+.1f s)%s", c.distance(), c.trueRemaining(),
                                    c.timingError(), c.late() ? " LATE" : "")
                            : String.format("callout %.0f m MISSED", c.distance());
                    track.events.add(new RunEvent(t, result.car.id, track.estimatorId, RunEvent.Type.CALLOUT, text));
                }
                // Reroute once the estimator is 90% sure the car left the route; re-arm when it drops below 50%.
                boolean[] r = rerouted.get(k);
                if (!r[0] && e.pOffRoute() >= cfg.nav.rerouteThreshold) {
                    r[0] = true;
                    track.events.add(new RunEvent(t, result.car.id, track.estimatorId, RunEvent.Type.REROUTE,
                            String.format("reroute (P off route %.2f)", e.pOffRoute())));
                } else if (r[0] && e.pOffRoute() < 0.5) {
                    r[0] = false;
                }
            }
        }
    }

    /** Tunnel entries/exits, GNSS outages and fork passages, from the truth and the log. */
    private static void truthEvents(CarResult result, RoadNetwork net, Route route) {
        TruthTrajectory truth = result.truth;
        String car = result.car.id;
        int prevTunnel = -1;
        int prevEdge = truth.edge(0);
        for (int i = 0; i < truth.size(); i++) {
            int tun = truth.tunnel(i);
            if (tun >= 0 && prevTunnel < 0) {
                result.events.add(new RunEvent(truth.t(i), car, null, RunEvent.Type.TUNNEL_ENTRY,
                        "enters tunnel " + (tun + 1)));
            } else if (tun < 0 && prevTunnel >= 0) {
                result.events.add(new RunEvent(truth.t(i), car, null, RunEvent.Type.TUNNEL_EXIT,
                        "leaves tunnel " + (prevTunnel + 1)));
            }
            prevTunnel = tun;
            // The car moved onto a new edge: it just passed a fork.
            int edge = truth.edge(i);
            if (edge != prevEdge) {
                boolean onRoute = route.contains(edge);
                result.forkPassages.add(new double[]{truth.t(i), onRoute ? 1 : 0, edge});
                result.events.add(new RunEvent(truth.t(i), car, null, RunEvent.Type.FORK,
                        "takes " + net.edge(edge).name + (onRoute ? " (on route)" : " (OFF route)")));
                prevEdge = edge;
            }
        }
        result.events.add(new RunEvent(truth.duration(), car, null, RunEvent.Type.ARRIVED, "end of run"));

        // GNSS outages: gaps of more than OUTAGE seconds between fixes.
        double lastFix = Double.NaN;
        for (GnssFix f : result.log.of(GnssFix.class)) {
            if (!Double.isNaN(lastFix) && f.t() - lastFix > OUTAGE) {
                result.gnssOutages.add(new double[]{lastFix, f.t()});
                result.events.add(new RunEvent(lastFix, car, null, RunEvent.Type.GNSS_LOST, "last GNSS fix"));
                result.events.add(new RunEvent(f.t(), car, null, RunEvent.Type.GNSS_BACK,
                        String.format("GNSS back after %.0f s", f.t() - lastFix)));
            }
            lastFix = f.t();
        }
    }
}
