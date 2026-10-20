package tunnelsim.metrics;

import tunnelsim.engine.CarResult;
import tunnelsim.engine.RunEvent;
import tunnelsim.engine.Track;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.nav.CalloutTracker.Callout;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Summary metrics of one estimator on one car (see docs/DESIGN.md §8). Errors are along-road,
 * signed (+ = estimate ahead of the car). NaN means "not applicable in this run".
 *
 * @param rmsTunnel        RMS error while the car is inside a tunnel [m]
 * @param maxTunnel        largest absolute error inside a tunnel [m]
 * @param p95Tunnel        95th percentile of absolute error inside a tunnel [m]
 * @param exitError        absolute error at the moment the car leaves each tunnel [m]
 * @param worstCallout     largest |timing error| of fired callouts [s]
 * @param lateCallouts     callouts that fired late or never
 * @param branchDecision   seconds after the car passes a fork until P(true branch) >= 0.9 for good
 * @param wrongBranchShown seconds the displayed marker sat on the wrong branch
 * @param rerouteLatency   seconds from leaving the route until the reroute event
 * @param falseReroute     a reroute was raised although the car stayed on the route
 * @param recovery         worst time from GNSS reacquisition until |error| < 10 m [s]
 * @param maxDisplayJump   largest jump of the displayed marker beyond the car's own motion [m]
 * @param within2Sigma     share of in-tunnel samples with |error| <= 2 sigma
 * @param nees             mean normalised estimation error squared in tunnels (1 = consistent)
 * @param runtimeMs        CPU time spent in the estimator [ms]
 */
public record TrackMetrics(String estimator, double rmsTunnel, double maxTunnel, double p95Tunnel,
                           double[] exitError, double worstCallout, int lateCallouts, double branchDecision,
                           double wrongBranchShown, double rerouteLatency, boolean falseReroute, double recovery,
                           double maxDisplayJump, double within2Sigma, double nees, double runtimeMs,
                           int reinitialisations) {

    /** Largest error over all tunnel exits in the run. */
    public double maxExitError() {
        return Arrays.stream(exitError).max().orElse(Double.NaN);
    }

    /** Computes all metrics of one estimator's track against the car's truth. */
    public static TrackMetrics compute(Track track, CarResult car, RoadNetwork net, Route route) {
        TruthTrajectory truth = car.truth;
        List<Double> tunnelErr = new ArrayList<>();
        List<Double> exitErr = new ArrayList<>();
        double sumSq = 0, nees = 0;
        int within = 0;
        double wrongShown = 0, maxJump = 0;
        Track.Sample prev = null;
        int prevTunnel = -1;
        double prevTruthOffset = 0;
        for (Track.Sample s : track.samples) {
            int i = truth.indexAt(s.t());
            int tun = truth.tunnel(i);
            // Inside a tunnel: collect the error, and check it against the estimator's own claimed uncertainty
            // (within 2 sigma? normalised squared error).
            if (tun >= 0 && s.estimate().valid() && !Double.isNaN(s.error())) {
                double e = s.error();
                tunnelErr.add(Math.abs(e));
                sumSq += e * e;
                double sig = Math.max(1.0, s.estimate().sigma());
                if (Math.abs(e) <= 2 * sig) {
                    within++;
                }
                nees += e * e / (sig * sig);
            }
            // The car just left a tunnel: the error of the last sample inside is the exit error.
            if (tun < 0 && prevTunnel >= 0 && prev != null && !Double.isNaN(prev.error())) {
                exitErr.add(Math.abs(prev.error()));
            }
            if (s.displayWrong()) {
                wrongShown += 0.1;
            }
            // Jump of the displayed marker = how much it moved beyond how much the car moved.
            if (prev != null && prev.displayLeaf() >= 0 && s.displayLeaf() >= 0) {
                double moved = s.displayOffset() - prev.displayOffset();
                double truthMoved = truth.offset(i) - prevTruthOffset;
                maxJump = Math.max(maxJump, Math.abs(moved - truthMoved));
            }
            prev = s;
            prevTunnel = tun;
            prevTruthOffset = truth.offset(i);
        }
        int nT = tunnelErr.size();
        // Sorted errors give the maximum and the 95th percentile.
        double[] sorted = tunnelErr.stream().mapToDouble(Double::doubleValue).sorted().toArray();

        // Callouts.
        double worst = Double.NaN;
        int late = 0;
        for (Callout c : track.callouts) {
            if (!c.fired() || c.late()) {
                late++;
            }
            if (c.fired()) {
                worst = Double.isNaN(worst) ? Math.abs(c.timingError()) : Math.max(worst, Math.abs(c.timingError()));
            }
        }

        // Branch decision and reroute.
        double decision = Double.NaN, reroute = Double.NaN;
        boolean falseReroute = false;
        double tOffRoute = Double.NaN;
        for (int fi = 0; fi < car.forkPassages.size(); fi++) {
            double[] fork = car.forkPassages.get(fi);
            double tf = fork[0];
            // The window ends at the next fork the car passes (or after 60 s).
            double windowEnd = fi + 1 < car.forkPassages.size()
                    ? Math.min(tf + 60, car.forkPassages.get(fi + 1)[0]) : tf + 60;
            boolean onRoute = fork[1] > 0;
            if (!onRoute && Double.isNaN(tOffRoute)) {
                tOffRoute = tf;
            }
            // Time until P(branch the car took at this fork) reaches 0.9 and stays there (until the
            // next fork). Taking the first moment above 0.9 would be flattering: while the particle
            // cloud still lags behind the car, all of it is before the fork, which counts as
            // consistent, and the decision only really happens once the cloud reaches the fork.
            // The per-fork probability ignores later forks, which the front of the cloud may reach
            // before the car does.
            double lastBelow = Double.NaN;
            for (Track.Sample s : track.samples) {
                if (s.t() >= tf && s.t() < windowEnd && s.estimate().valid() && s.pTruthBranch() < 0.9) {
                    lastBelow = s.t();
                }
            }
            double dt = Double.isNaN(lastBelow) ? 0 : lastBelow + tunnelsim.engine.RunConstants.RECORD_DT - tf;
            decision = Double.isNaN(decision) ? dt : Math.max(decision, dt);
        }
        // Reroute latency: time from leaving the route to the reroute event. A reroute while the car is still on
        // the route is a false alarm.
        for (RunEvent ev : track.events) {
            if (ev.type() == RunEvent.Type.REROUTE) {
                if (Double.isNaN(tOffRoute) || ev.t() < tOffRoute - 1) {
                    falseReroute = true;
                } else if (Double.isNaN(reroute)) {
                    reroute = ev.t() - tOffRoute;
                }
            }
        }

        // Recovery after each outage.
        double recovery = Double.NaN;
        // Recovery: after each GNSS outage, time until the error is back under 10 m on the right branch.
        for (double[] outage : car.gnssOutages) {
            double back = outage[1];
            double rec = Double.NaN;
            for (Track.Sample s : track.samples) {
                if (s.t() >= back && !Double.isNaN(s.error()) && Math.abs(s.error()) < 10 && !s.wrongBranch()) {
                    rec = s.t() - back;
                    break;
                }
            }
            if (Double.isNaN(rec)) {
                rec = truth.duration() - back;
            }
            recovery = Double.isNaN(recovery) ? rec : Math.max(recovery, rec);
        }

        return new TrackMetrics(track.estimatorId,
                nT == 0 ? Double.NaN : Math.sqrt(sumSq / nT),
                nT == 0 ? Double.NaN : sorted[nT - 1],
                nT == 0 ? Double.NaN : sorted[(int) Math.min(nT - 1, Math.floor(0.95 * nT))],
                exitErr.stream().mapToDouble(Double::doubleValue).toArray(),
                worst, late, decision, wrongShown, reroute, falseReroute, recovery, maxJump,
                nT == 0 ? Double.NaN : (double) within / nT,
                nT == 0 ? Double.NaN : nees / nT,
                track.nanos / 1e6, track.reinitialisations);
    }
}
