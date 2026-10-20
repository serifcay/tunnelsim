package tunnelsim.app;

import javafx.geometry.VPos;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.TextAlignment;
import tunnelsim.engine.CarResult;
import tunnelsim.engine.Track;
import tunnelsim.map.Edge;
import tunnelsim.map.RoadNetwork;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.Locale;

/**
 * The road straightened out around the car: along-road error is what matters in a single-lane
 * tunnel, and at tunnel scale it is invisible on the map. One row per estimator with its +-2 sigma.
 */
final class StripView extends CanvasPane {
    private static final double LABEL = 70;
    private final AppState state;
    private double halfWindow = 300;

    StripView(AppState state) {
        this.state = state;
        setOnScroll(e -> {
            // Scroll wheel changes how many metres around the car are shown (25 m to 5 km each side).
            halfWindow = Math.max(25, Math.min(5000, halfWindow * Math.pow(0.998, e.getDeltaY())));
            redraw();
        });
    }

    @Override
    void redraw() {
        GraphicsContext g = g();
        g.setFill(Palette.PANEL);
        g.fillRect(0, 0, w(), h());
        CarResult car = state.car();
        if (car == null) {
            return;
        }
        RoadNetwork net = state.result.net();
        TruthTrajectory truth = car.truth;
        int ti = truth.indexAt(state.time);
        double center = truth.offset(ti);
        double x0 = LABEL, x1 = w() - 12;
        double lo = center - halfWindow, hi = center + halfWindow;
        // The visible stretch of road is centred on the car; px converts a path offset [m] to a screen x [px].
        java.util.function.DoubleUnaryOperator px = off -> x0 + (off - lo) / (hi - lo) * (x1 - x0);

        // One row for the truth plus one per visible estimator.
        int rows = 1 + (int) state.visible.stream().filter(car.tracks::containsKey).count();
        double top = 26, rowH = Math.max(18, Math.min(34, (h() - top - 8) / rows));

        // Tunnels along the true path.
        for (int e = truth.leaf; e >= 0; e = net.edge(e).parent) {
            Edge edge = net.edge(e);
            for (double[] iv : net.tunnelIntervals(e)) {
                double a = px.applyAsDouble(edge.pathStart + iv[0]), b = px.applyAsDouble(edge.pathStart + iv[1]);
                g.setFill(Color.web("#e8e6e1"));
                g.fillRect(Math.max(x0, a), top - 6, Math.max(0, Math.min(x1, b) - Math.max(x0, a)), h() - top);
            }
        }
        // Grid with distance labels.
        double step = niceStep(halfWindow / 3);
        g.setFont(Font.font("System", 10));
        g.setTextAlign(TextAlignment.CENTER);
        g.setTextBaseline(VPos.TOP);
        for (double m = Math.ceil(lo / step) * step; m <= hi; m += step) {
            double x = px.applyAsDouble(m);
            g.setStroke(Palette.GRID);
            g.setLineWidth(1);
            g.strokeLine(x, top - 6, x, h());
            g.setFill(Palette.MUTED);
            g.fillText(String.format(Locale.ROOT, "%.0f m", m), x, 4);
        }
        // Forks and the maneuver with its callout distances.
        for (double fork : net.forks()) {
            double x = px.applyAsDouble(fork);
            g.setStroke(Color.web("#c0392b"));
            g.setLineDashes(4, 3);
            g.strokeLine(x, top - 6, x, h());
            g.setLineDashes();
        }
        if (state.result.route().hasManeuver()) {
            double m = state.result.route().maneuverOffset();
            for (double d : state.result.scenario().nav.calloutDistances) {
                double x = px.applyAsDouble(m - d);
                if (x >= x0 && x <= x1) {
                    g.setStroke(Color.web("#c0392b"));
                    g.setLineWidth(1);
                    g.strokeLine(x, top - 6, x, top + 2);
                    g.setFill(Color.web("#c0392b"));
                    g.fillText(String.format(Locale.ROOT, "%.0f", d), x, top + 3);
                }
            }
        }

        // Truth row.
        double y = top + rowH * 0.5;
        g.setTextAlign(TextAlignment.LEFT);
        g.setTextBaseline(VPos.CENTER);
        g.setFont(Font.font("System", 11));
        g.setFill(Palette.TRUTH);
        g.fillText("truth", 8, y);
        double tx = px.applyAsDouble(center);
        g.fillPolygon(new double[]{tx - 6, tx + 6, tx}, new double[]{y - 6, y - 6, y + 6}, 3);
        g.setStroke(Palette.withAlpha(Palette.TRUTH, 0.4));
        g.setLineWidth(1);
        g.strokeLine(tx, top - 6, tx, h());

        int row = 1;
        for (String id : state.visible) {
            Track track = car.tracks.get(id);
            if (track == null) {
                continue;
            }
            y = top + rowH * (row + 0.5);
            row++;
            Color c = Palette.estimator(id);
            g.setFill(c);
            g.fillText(id, 8, y);
            Track.Sample s = track.at(state.time);
            if (s == null || !s.estimate().valid() || Double.isNaN(s.error())) {
                g.setFill(Palette.MUTED);
                g.fillText("no estimate", x0 + 6, y);
                continue;
            }
            // Place the estimate at the car's position plus its signed error, with a bar for its +-2 sigma.
            double off = center + s.error();
            double sig = 2 * s.estimate().sigma();
            double a = Math.max(x0, px.applyAsDouble(off - sig)), b = Math.min(x1, px.applyAsDouble(off + sig));
            if (b > a) {
                g.setFill(Palette.withAlpha(c, 0.22));
                g.fillRoundRect(a, y - rowH * 0.3, b - a, rowH * 0.6, 6, 6);
            }
            double ex = px.applyAsDouble(off);
            // Estimate outside the visible window: draw an arrow at the edge pointing towards it.
            if (ex < x0 || ex > x1) {
                g.setFill(c);
                double edgeX = ex < x0 ? x0 + 4 : x1 - 4;
                double dir = ex < x0 ? -1 : 1;
                g.fillPolygon(new double[]{edgeX, edgeX - dir * 8, edgeX - dir * 8},
                        new double[]{y, y - 5, y + 5}, 3);
            } else {
                g.setFill(c);
                g.fillPolygon(new double[]{ex, ex + 6, ex, ex - 6}, new double[]{y - 6, y, y + 6, y}, 4);
            }
            if (state.showDisplayMarker && !Double.isNaN(s.displayError())) {
                double dx = px.applyAsDouble(center + s.displayError());
                g.setStroke(c);
                g.setLineWidth(1.5);
                g.strokeOval(dx - 6, y - 6, 12, 12);
            }
            // Text: signed error, +-2 sigma, and a warning if the estimate is on the wrong branch.
            String label = String.format(Locale.ROOT, "%+.1f m  (±%.0f)%s", s.error(), sig,
                    s.wrongBranch() ? "  WRONG BRANCH" : "");
            g.setFill(s.wrongBranch() ? Color.web("#c0392b") : Palette.TEXT);
            double lx = Math.min(Math.max(ex, x0) + 10, x1 - label.length() * 6.2);
            g.fillText(label, lx, y);
        }
        g.setTextBaseline(VPos.BASELINE);
    }

    private static double niceStep(double m) {
        double p = Math.pow(10, Math.floor(Math.log10(m)));
        double f = m / p;
        return (f < 2 ? 1 : f < 5 ? 2 : 5) * p;
    }
}
