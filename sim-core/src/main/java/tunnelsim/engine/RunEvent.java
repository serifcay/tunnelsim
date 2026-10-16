package tunnelsim.engine;

/** Something noteworthy during a run, for the event log and the metrics. */
public record RunEvent(double t, String car, String estimator, Type type, String text) {
    public enum Type { TUNNEL_ENTRY, TUNNEL_EXIT, GNSS_LOST, GNSS_BACK, FORK, CALLOUT, REROUTE, BRANCH_DECIDED, ARRIVED }
}
