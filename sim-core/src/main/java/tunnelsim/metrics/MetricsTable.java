package tunnelsim.metrics;

import tunnelsim.engine.CarResult;
import tunnelsim.engine.RunResult;

import java.util.Locale;

/** Plain-text summary of a run, one row per car and estimator. */
public final class MetricsTable {
    private MetricsTable() {
    }

    public static String format(RunResult run) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.ROOT, "%s  (computed in %.2f s)%n", run.scenario().name, run.computeSeconds()));
        sb.append(String.format(Locale.ROOT, "%-10s %-4s %8s %8s %8s %8s %9s %5s %8s %8s %8s %7s %6s %7s%n",
                "car", "est", "rmsTun", "maxTun", "p95Tun", "exitErr", "callout", "late", "branch", "reroute",
                "recover", "jump", "in2sig", "ms"));
        // One row per car and estimator, columns as in the header above.
        for (CarResult car : run.cars()) {
            for (TrackMetrics m : car.metrics.values()) {
                sb.append(String.format(Locale.ROOT, "%-10s %-4s %8s %8s %8s %8s %9s %5d %8s %8s %8s %7s %6s %7.0f%n",
                        car.car.id, m.estimator(), f(m.rmsTunnel()), f(m.maxTunnel()), f(m.p95Tunnel()),
                        f(m.maxExitError()), f(m.worstCallout()), m.lateCallouts(), f(m.branchDecision()),
                        m.falseReroute() ? "FALSE" : f(m.rerouteLatency()), f(m.recovery()), f(m.maxDisplayJump()),
                        f(m.within2Sigma()), m.runtimeMs()));
            }
        }
        return sb.toString();
    }

    /** Number formatting for the table: '-' for not applicable, no decimals for large values. */
    private static String f(double v) {
        if (Double.isNaN(v)) {
            return "-";
        }
        return Math.abs(v) >= 100 ? String.format(Locale.ROOT, "%.0f", v) : String.format(Locale.ROOT, "%.1f", v);
    }
}
