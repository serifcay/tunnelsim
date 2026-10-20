package tunnelsim.vehicle;

import tunnelsim.map.Edge;
import tunnelsim.map.MapPosition;
import tunnelsim.map.Pose;
import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.ScenarioConfig.CarConfig;
import tunnelsim.scenario.ScenarioConfig.DriverConfig;
import tunnelsim.scenario.ScenarioConfig.EventConfig;
import tunnelsim.util.GaussMarkov;
import tunnelsim.util.MathUtil;
import tunnelsim.util.Rng;
import tunnelsim.util.Rot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Generates a car's ground truth trajectory: a kinematic point mass following the road, with a
 * speed controller that respects speed limits, curve speeds and scripted traffic events.
 */
public final class Driver {
    public static final double DT = 0.01;

    private static final double SQUAT_PITCH = 0.0035; // rad per m/s^2 of longitudinal acceleration
    private static final double ROLL_GAIN = 0.008;    // rad per m/s^2 of lateral acceleration
    /**
     * Suspension response time [s]. Body pitch and roll follow acceleration through this lag; without
     * it, the instant the car stops the braking dive would snap back within one 10 ms step (~1.4 rad/s
     * of pitch rate), which no real car does and which the aligner would mistake for phone handling.
     */
    private static final double SUSPENSION_TAU = 0.3;
    private static final double SPEED_GAIN = 0.8;     // 1/s
    private static final double JERK = 2.5;           // m/s^3
    private static final double HARD_JERK = 15;       // m/s^3

    private final RoadNetwork net;
    private final CarConfig car;
    private final DriverConfig dc;
    private final Rng rng;
    // The branch this car will actually drive (decided once at the start), and where it ends.
    private final int leaf;
    private final double pathEnd;

    // Event state.
    private final List<EventConfig> events;
    private final boolean[] done;
    private final double[] reachedAt;
    // State of the scripted events: currently held stop, hard brake in progress, stop-and-go phase.
    private double holdUntil = -1;
    private int holdingEvent = -1;
    private int hardBrakeEvent = -1;
    private double stopAndGoPhaseStart = -1;
    private boolean stopAndGoStopping;

    public Driver(RoadNetwork net, Route route, CarConfig car, Rng rng) {
        this.net = net;
        this.car = car;
        this.dc = car.driver;
        this.rng = rng;
        this.leaf = choosePath(route);
        this.pathEnd = net.pathEnd(leaf);
        this.events = new ArrayList<>(car.events);
        this.events.sort(Comparator.comparingDouble(e -> e.at));
        this.done = new boolean[events.size()];
        this.reachedAt = new double[events.size()];
        java.util.Arrays.fill(reachedAt, Double.NaN);
    }

    /** Decides at every fork whether this driver follows the route. */
    private int choosePath(Route route) {
        int e = 0;
        while (!net.edge(e).isLeaf()) {
            // With probability pFollowRoute take the route's branch, otherwise the other one.
            int routeChild = route.next(e);
            List<Integer> children = net.edge(e).children();
            if (rng.chance(dc.pFollowRoute)) {
                e = routeChild;
            } else {
                e = children.get(0) == routeChild ? children.get(1) : children.get(0);
            }
        }
        return e;
    }

    /** Drives the whole run in 10 ms steps and records the true state at every step. */
    public TruthTrajectory simulate(double duration) {
        TruthTrajectory traj = new TruthTrajectory(DT, leaf, (int) (duration / DT) + 2);
        double[] mount = initialMount();
        traj.addMountChange(new TruthTrajectory.MountChange(0, mount));

        // Drivers don't hold an exact speed: a slow random fluctuation around the target.
        GaussMarkov jitter = new GaussMarkov(dc.speedJitter, 20, rng);
        double off = 0, v = car.startSpeed, a = 0;
        double squat = 0, lean = 0;
        int steps = (int) Math.round(duration / DT);
        for (int k = 0; k <= steps; k++) {
            double t = k * DT;
            // Where the car is now, and the road geometry there.
            MapPosition pos = net.positionOnPath(leaf, off);
            Edge edge = net.edge(pos.edge());
            Pose pose = edge.pose(pos.s());
            double latAcc = v * v * pose.curvature();
            // Body attitude: the nose lifts when accelerating and dips when braking (squat), the body leans out in
            // curves, both through the suspension lag. Pitch also includes the road slope.
            squat += (SQUAT_PITCH * a - squat) * DT / SUSPENSION_TAU;
            lean += (ROLL_GAIN * latAcc - lean) * DT / SUSPENSION_TAU;
            double pitch = Math.atan(pose.grade()) + squat;
            double roll = lean;
            // Record the true state for this step.
            traj.add(t, pos.edge(), pos.s(), off, pose.x(), pose.y(), pose.z(), pose.heading(), pitch, roll,
                    v, a, pose.curvature(), pose.grade(), net.tunnelAt(pos));

            if (off >= pathEnd - 0.5) {
                break; // arrived
            }

            // Mount shift events (optionally some seconds after reaching the position).
            for (int i = 0; i < events.size(); i++) {
                EventConfig e = events.get(i);
                if (!done[i] && "mountShift".equals(e.type) && off >= e.at && Double.isNaN(reachedAt[i])) {
                    reachedAt[i] = t;
                }
                if (!done[i] && "mountShift".equals(e.type) && !Double.isNaN(reachedAt[i])
                        && t >= reachedAt[i] + e.afterSeconds) {
                    done[i] = true;
                    // The phone rotates smoothly over half a second, so the gyroscope sees the motion.
                    double[] start = mount;
                    int n = 50;
                    // Store the rotation as 50 small steps, one per 10 ms.
                    for (int j = 1; j <= n; j++) {
                        double f = (double) j / n;
                        double[] delta = Rot.mul(Rot.axisAngle(new double[]{1, 0, 0}, Math.toRadians(f * e.pitchDeg)),
                                Rot.mul(Rot.axisAngle(new double[]{0, 1, 0}, Math.toRadians(f * e.yawDeg)),
                                        Rot.axisAngle(new double[]{0, 0, 1}, Math.toRadians(f * e.rollDeg))));
                        mount = Rot.mul(delta, start);
                        traj.addMountChange(new TruthTrajectory.MountChange(t + j * DT, mount));
                    }
                }
            }

            double jerk = JERK;
            double aCmd;
            // Before the start time the car stands still (the phone can learn "up" meanwhile).
            if (t < car.startTime) {
                aCmd = 0;
                v = 0;
                a = 0;
            // Holding at a stop until its time is up; then the event is done and the car pulls away.
            } else if (holdingEvent >= 0) {
                v = 0;
                a = 0;
                aCmd = 0;
                if (t >= holdUntil) {
                    if ("stop".equals(events.get(holdingEvent).type)) {
                        done[holdingEvent] = true;
                    }
                    holdingEvent = -1;
                    stopAndGoPhaseStart = t;
                    stopAndGoStopping = false;
                }
            } else {
                // Normal driving: steer the speed towards the desired speed (a simple proportional controller).
                double vDes = desiredSpeed(off, v, t);
                // Speed fluctuation scales down at low target speeds so stops are real stops.
                vDes = Math.max(0, vDes + jitter.step(DT) * Math.min(1, vDes / 10));
                aCmd = MathUtil.clamp(SPEED_GAIN * (vDes - v), -4.0, dc.maxAccel);
                // The speed loop lags a braking curve; brake directly for a stop target ahead.
                double stopAt = nextStopPosition(off);
                if (!Double.isNaN(stopAt) && v > 0) {
                    double d = stopAt - off;
                    double aReq = d > 0.5 ? v * v / (2 * d) : 4.0;
                    if (aReq > 0.6 * dc.comfortDecel) {
                        aCmd = Math.min(aCmd, -Math.min(6.0, 1.1 * aReq));
                        jerk = HARD_JERK;
                    }
                }
                // Hard brake in progress: brake hard until the target speed is reached.
                if (hardBrakeEvent >= 0) {
                    EventConfig e = events.get(hardBrakeEvent);
                    if (v > e.speed + 0.2) {
                        aCmd = -e.decel;
                        jerk = HARD_JERK;
                    } else {
                        done[hardBrakeEvent] = true;
                        hardBrakeEvent = -1;
                    }
                }
                // Enter a hold when a stop target is reached.
                int stopEvent = pendingStop(off);
                if (stopEvent >= 0 && v < 0.4) {
                    EventConfig e = events.get(stopEvent);
                    holdingEvent = stopEvent;
                    holdUntil = t + ("stop".equals(e.type) ? e.duration : e.stopTime);
                    v = 0;
                    a = 0;
                    aCmd = 0;
                }
            }
            // Acceleration may only change gradually (limited jerk), then integrate into speed and distance.
            a += MathUtil.clamp(aCmd - a, -jerk * DT, jerk * DT);
            double vNew = Math.max(0, v + a * DT);
            off += 0.5 * (v + vNew) * DT;
            v = vNew;
            if (v == 0 && a < 0) {
                a = 0;
            }
        }
        return traj;
    }

    /** Random holder orientation for this run: the nominal portrait mount, turned by small random angles. */
    private double[] initialMount() {
        // Nominal portrait dashboard mount: phone x = car right, phone y = car up, phone z = car backwards.
        double[] nominal = {0, -1, 0, 0, 0, 1, -1, 0, 0};
        double yaw = Math.toRadians(rng.inRange(car.mount.yawDeg));
        double pitch = Math.toRadians(rng.inRange(car.mount.pitchDeg));
        double roll = Math.toRadians(rng.inRange(car.mount.rollDeg));
        double[] delta = Rot.mul(Rot.axisAngle(new double[]{1, 0, 0}, pitch),
                Rot.mul(Rot.axisAngle(new double[]{0, 1, 0}, yaw), Rot.axisAngle(new double[]{0, 0, 1}, roll)));
        return Rot.mul(delta, nominal);
    }

    /** Speed the driver aims for, considering limits, curves and events ahead. */
    private double desiredSpeed(double off, double v, double t) {
        double decel = dc.comfortDecel;
        // Look ahead far enough to brake comfortably for anything coming up.
        double lookahead = Math.max(60, v * v / (2 * decel) + 3 * v);
        double vDes = Double.MAX_VALUE;
        // Every 5 m ahead: the speed limit there, lowered in curves so sideways acceleration stays comfortable.
        // The speed allowed now is the speed from which we could still slow down to that limit in time.
        for (double d = 0; d <= lookahead; d += 5) {
            MapPosition p = net.positionOnPath(leaf, off + d);
            Edge e = net.edge(p.edge());
            double limit = e.speedLimit * dc.speedFactor;
            double k = Math.abs(e.curvature(p.s()));
            if (k > 1e-6) {
                limit = Math.min(limit, Math.sqrt(dc.maxLateralAccel / k));
            }
            vDes = Math.min(vDes, Math.sqrt(limit * limit + 2 * decel * d));
        }
        // End of the road: come to a stop.
        vDes = Math.min(vDes, Math.sqrt(2 * decel * Math.max(0, pathEnd - off - 1)));

        for (int i = 0; i < events.size(); i++) {
            EventConfig e = events.get(i);
            if (done[i]) {
                continue;
            }
            // Scripted traffic events can lower the desired speed further. d = distance to the event.
            double d = e.at - off;
            switch (e.type) {
                // Slow zone: approach at a braking curve, hold the speed inside, done after it.
                case "slowTo" -> {
                    if (off >= e.at && off <= e.at + e.length) {
                        vDes = Math.min(vDes, e.speed);
                    } else if (d > 0) {
                        vDes = Math.min(vDes, Math.sqrt(e.speed * e.speed + 2 * decel * d));
                    } else if (off > e.at + e.length) {
                        done[i] = true;
                    }
                }
                // Stop: brake to come to rest at the stop position.
                case "stop" -> vDes = Math.min(vDes, Math.sqrt(2 * decel * Math.max(0, d)));
                // Stop-and-go zone: drive for goTime seconds, stop, wait stopTime seconds, repeat until the end of
                // the zone.
                case "stopAndGo" -> {
                    if (d > 0) {
                        vDes = Math.min(vDes, Math.sqrt(e.speed * e.speed + 2 * decel * d));
                    } else if (off <= e.at + e.length) {
                        if (stopAndGoPhaseStart < 0) {
                            stopAndGoPhaseStart = t;
                            stopAndGoStopping = false;
                        }
                        if (!stopAndGoStopping && t - stopAndGoPhaseStart > e.goTime) {
                            stopAndGoStopping = true;
                        }
                        vDes = Math.min(vDes, stopAndGoStopping ? 0 : e.speed);
                    } else {
                        done[i] = true;
                    }
                }
                // Hard brake: starts once the car reaches the position.
                case "hardBrake" -> {
                    if (d <= 0 && hardBrakeEvent < 0) {
                        hardBrakeEvent = i;
                    }
                }
                default -> {
                }
            }
        }
        return vDes;
    }

    /** Position of the next pending "stop" event, or NaN. */
    private double nextStopPosition(double off) {
        for (int i = 0; i < events.size(); i++) {
            EventConfig e = events.get(i);
            if (!done[i] && "stop".equals(e.type) && e.at - off < 300) {
                return e.at;
            }
        }
        return Double.NaN;
    }

    /** An event whose stop target has been reached: a "stop" at its position or a stop-and-go stop phase. */
    private int pendingStop(double off) {
        for (int i = 0; i < events.size(); i++) {
            EventConfig e = events.get(i);
            if (done[i]) {
                continue;
            }
            if ("stop".equals(e.type) && off >= e.at - 3) {
                return i;
            }
            if ("stopAndGo".equals(e.type) && stopAndGoStopping && off >= e.at && off <= e.at + e.length) {
                return i;
            }
        }
        return -1;
    }

    public int leaf() {
        return leaf;
    }
}
