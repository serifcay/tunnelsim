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
 * @param branchDecision   seconds after the car passes a fork until P(true branch) >= 0.9
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

    public double maxExitError() {
        return Arrays.stream(exitError).max().orElse(Double.NaN);
    }

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
            if (tun < 0 && prevTunnel >= 0 && prev != null && !Double.isNaN(prev.error())) {
                exitErr.add(Math.abs(prev.error()));
            }
            if (s.displayWrong()) {
                wrongShown += 0.1;
            }
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
        for (double[] fork : car.forkPassages) {
            double tf = fork[0];
            boolean onRoute = fork[1] > 0;
            if (!onRoute && Double.isNaN(tOffRoute)) {
                tOffRoute = tf;
            }
            for (Track.Sample s : track.samples) {
                if (s.t() >= tf && s.pTruthPath() >= 0.9) {
                    decision = Double.isNaN(decision) ? s.t() - tf : Math.max(decision, s.t() - tf);
                    break;
                }
            }
        }
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
