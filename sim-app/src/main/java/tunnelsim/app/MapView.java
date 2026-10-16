package tunnelsim.app;

import javafx.geometry.VPos;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.shape.StrokeLineCap;
import javafx.scene.text.Font;
import javafx.scene.text.TextAlignment;
import tunnelsim.engine.CarResult;
import tunnelsim.engine.Track;
import tunnelsim.estimator.Estimate;
import tunnelsim.map.Edge;
import tunnelsim.map.Features.Beacon;
import tunnelsim.map.Features.CellZone;
import tunnelsim.map.Features.Portal;
import tunnelsim.map.MapPosition;
import tunnelsim.map.RoadNetwork;
import tunnelsim.sensor.SensorEvent.GnssFix;
import tunnelsim.vehicle.TruthTrajectory;

import java.util.List;
import java.util.Locale;

/** Top view of the road network with the true car, estimates, uncertainty and particles. */
final class MapView extends CanvasPane {
    private final AppState state;
    private double cx, cy, scale = 1;
    private boolean follow = true;
    private double dragX, dragY;
    private List<GnssFix> fixes = List.of();

    MapView(AppState state) {
        this.state = state;
        setOnScroll(e -> {
            double f = Math.pow(1.0015, e.getDeltaY());
            double wx = toWorldX(e.getX()), wy = toWorldY(e.getY());
            scale = Math.max(0.01, Math.min(50, scale * f));
            if (!follow) {
                cx = wx - (e.getX() - w() / 2) / scale;
                cy = wy + (e.getY() - h() / 2) / scale;
            }
            redraw();
        });
        setOnMousePressed(e -> {
            dragX = e.getX();
            dragY = e.getY();
        });
        setOnMouseDragged(e -> {
            follow = false;
            cx -= (e.getX() - dragX) / scale;
            cy += (e.getY() - dragY) / scale;
            dragX = e.getX();
            dragY = e.getY();
            redraw();
        });
        setOnMouseClicked(e -> {
            if (e.getClickCount() == 2) {
                follow = !follow;
                if (!follow) {
                    fitAll();
                }
                redraw();
            }
        });
    }

    void resultChanged() {
        CarResult car = state.car();
        fixes = car == null ? List.of() : car.log.of(GnssFix.class);
        if (scale == 1) {
            scale = 0.9; // first run: ~900 m across a 800 px view
        }
    }

    void setFollow(boolean f) {
        follow = f;
        if (!f) {
            fitAll();
        }
        redraw();
    }

    boolean follow() {
        return follow;
    }

    void zoom(double factor) {
        scale = Math.max(0.01, Math.min(50, scale * factor));
        redraw();
    }

    void fitAll() {
        if (state.result == null) {
            return;
        }
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (Edge e : state.result.net().edges()) {
            for (double s = 0; s <= e.length; s += 10) {
                minX = Math.min(minX, e.x(s));
                maxX = Math.max(maxX, e.x(s));
                minY = Math.min(minY, e.y(s));
                maxY = Math.max(maxY, e.y(s));
            }
        }
        cx = (minX + maxX) / 2;
        cy = (minY + maxY) / 2;
        scale = Math.min(w() / Math.max(1, maxX - minX), h() / Math.max(1, maxY - minY)) * 0.9;
    }

    private double sx(double x) {
        return w() / 2 + (x - cx) * scale;
    }

    private double sy(double y) {
        return h() / 2 - (y - cy) * scale;
    }

    private double toWorldX(double px) {
        return cx + (px - w() / 2) / scale;
    }

    private double toWorldY(double py) {
        return cy - (py - h() / 2) / scale;
    }

    @Override
    void redraw() {
        GraphicsContext g = g();
        g.setFill(Palette.BACKGROUND);
        g.fillRect(0, 0, w(), h());
        if (state.result == null) {
            g.setFill(Palette.MUTED);
            g.fillText("Running...", 20, 30);
            return;
        }
        RoadNetwork net = state.result.net();
        CarResult car = state.car();
        TruthTrajectory truth = car.truth;
        int ti = truth.indexAt(state.time);
        if (follow) {
            cx = truth.x(ti);
            cy = truth.y(ti);
        }

        // Roads, then tunnels on top.
        g.setLineCap(StrokeLineCap.ROUND);
        double roadWidth = Math.max(3, 8 * scale);
        for (Edge e : net.edges()) {
            strokeEdge(g, e, 0, e.length, Palette.ROAD_EDGE, roadWidth + 2);
            strokeEdge(g, e, 0, e.length, Palette.ROAD, roadWidth);
        }
        for (Edge e : net.edges()) {
            for (double[] iv : net.tunnelIntervals(e.id)) {
                strokeEdge(g, e, iv[0], iv[1], Palette.TUNNEL, roadWidth + 6);
            }
        }
        g.setLineDashes();
        for (Portal p : net.portals()) {
            g.setFill(Color.web("#f0c040"));
            g.fillOval(sx(p.x()) - 4, sy(p.y()) - 4, 8, 8);
        }
        for (CellZone c : net.cells()) {
            g.setStroke(Color.web("#6a8fd8"));
            g.setLineWidth(1.5);
            g.strokeLine(sx(c.x()) - 5, sy(c.y()) + 9, sx(c.x()) + 5, sy(c.y()) + 9);
        }
        if (state.result.scenario().options.ble) {
            for (Beacon b : net.beacons()) {
                double x = sx(b.x()), y = sy(b.y());
                g.setFill(b.failed() ? Palette.MUTED : Palette.BEACON);
                g.fillRect(x - 2.5, y - 2.5, 5, 5);
                if (b.failed()) {
                    g.setStroke(Color.web("#c0392b"));
                    g.setLineWidth(1);
                    g.strokeLine(x - 4, y - 4, x + 4, y + 4);
                }
            }
        }

        // Recent GNSS fixes.
        for (GnssFix f : fixes) {
            if (f.t() > state.time || f.t() < state.time - 15) {
                continue;
            }
            double a = 0.25 + 0.75 * (1 - (state.time - f.t()) / 15);
            g.setStroke(Palette.withAlpha(Palette.GNSS, a));
            g.setLineWidth(1.5);
            double x = sx(f.x()), y = sy(f.y());
            g.strokeLine(x - 4, y - 4, x + 4, y + 4);
            g.strokeLine(x - 4, y + 4, x + 4, y - 4);
        }

        // Other cars (truth only).
        for (int k = 0; k < state.result.cars().size(); k++) {
            if (k == state.car) {
                continue;
            }
            TruthTrajectory other = state.result.cars().get(k).truth;
            if (state.time > other.duration()) {
                continue;
            }
            int oi = other.indexAt(state.time);
            drawCar(g, other.x(oi), other.y(oi), other.heading(oi), Palette.MUTED, 0.8);
        }

        // Estimates of the selected car.
        int lane = 0;
        for (String id : state.visible) {
            Track track = car.tracks.get(id);
            if (track == null) {
                continue;
            }
            Track.Sample s = track.at(state.time);
            if (s == null || !s.estimate().valid()) {
                continue;
            }
            Estimate e = s.estimate();
            Color c = Palette.estimator(id);
            if (state.showParticles && e.particles() != null) {
                g.setFill(Palette.withAlpha(c, 0.35));
                Estimate.ParticleSnapshot ps = e.particles();
                for (int k = 0; k < ps.edges().length; k++) {
                    Edge pe = net.edge(ps.edges()[k]);
                    g.fillOval(sx(pe.x(ps.s()[k])) - 1.5, sy(pe.y(ps.s()[k])) - 1.5, 3, 3);
                }
            }
            // +-2 sigma along the road, as a thin line beside the road (one lane per estimator).
            double sig = Math.min(2 * e.sigma(), 3000);
            lane++;
            if (sig * scale > 2) {
                g.setStroke(Palette.withAlpha(c, 0.75));
                g.setLineWidth(3);
                g.beginPath();
                int steps = 60;
                double side = roadWidth / 2 + 4 + 5 * lane;
                for (int k = 0; k <= steps; k++) {
                    double off = e.offset() - sig + 2 * sig * k / steps;
                    MapPosition p = net.positionOnPath(e.leaf(), off);
                    Edge pe = net.edge(p.edge());
                    double hd = pe.pose(p.s()).heading();
                    double x = sx(pe.x(p.s())) + side * Math.sin(hd), y = sy(pe.y(p.s())) + side * Math.cos(hd);
                    if (k == 0) {
                        g.moveTo(x, y);
                    } else {
                        g.lineTo(x, y);
                    }
                }
                g.stroke();
            }
            Edge ee = net.edge(e.edge());
            double ex = sx(ee.x(e.s())), ey = sy(ee.y(e.s()));
            g.setFill(c);
            g.fillPolygon(new double[]{ex, ex + 7, ex, ex - 7}, new double[]{ey - 7, ey, ey + 7, ey}, 4);
            g.setStroke(Color.WHITE);
            g.setLineWidth(1.2);
            g.strokePolygon(new double[]{ex, ex + 7, ex, ex - 7}, new double[]{ey - 7, ey, ey + 7, ey}, 4);
            g.setFill(c);
            g.setFont(Font.font("System", 11));
            g.fillText(id, ex + 9, ey - 6);
            if (state.showDisplayMarker && s.displayLeaf() >= 0) {
                MapPosition dp = net.positionOnPath(s.displayLeaf(), s.displayOffset());
                Edge de = net.edge(dp.edge());
                g.setStroke(c);
                g.setLineWidth(2);
                g.strokeOval(sx(de.x(dp.s())) - 9, sy(de.y(dp.s())) - 9, 18, 18);
            }
        }

        // The true car on top.
        drawCar(g, truth.x(ti), truth.y(ti), truth.heading(ti), Palette.TRUTH, 1.0);
        drawOverlay(g, car, ti);
    }

    private void strokeEdge(GraphicsContext g, Edge e, double from, double to, Color c, double width) {
        g.setStroke(c);
        g.setLineWidth(width);
        g.beginPath();
        double step = Math.max(2, 3 / scale);
        boolean first = true;
        for (double s = from; ; s += step) {
            double ss = Math.min(s, to);
            double x = sx(e.x(ss)), y = sy(e.y(ss));
            if (first) {
                g.moveTo(x, y);
                first = false;
            } else {
                g.lineTo(x, y);
            }
            if (ss >= to) {
                break;
            }
        }
        g.stroke();
    }

    private void drawCar(GraphicsContext g, double x, double y, double heading, Color c, double size) {
        double px = sx(x), py = sy(y);
        double len = 11 * size, wid = 6 * size;
        double ch = Math.cos(heading), sh = Math.sin(heading);
        double[] xs = {px + ch * len, px - ch * len * 0.6 - sh * wid, px - ch * len * 0.6 + sh * wid};
        double[] ys = {py - sh * len, py + sh * len * 0.6 - ch * wid, py + sh * len * 0.6 + ch * wid};
        g.setFill(c);
        g.fillPolygon(xs, ys, 3);
        g.setStroke(Color.WHITE);
        g.setLineWidth(1.5);
        g.strokePolygon(xs, ys, 3);
    }

    private void drawOverlay(GraphicsContext g, CarResult car, int ti) {
        TruthTrajectory truth = car.truth;
        RoadNetwork net = state.result.net();
        g.setFont(Font.font("System", 12));
        g.setTextAlign(TextAlignment.LEFT);
        g.setTextBaseline(VPos.TOP);
        String where;
        int tun = truth.tunnel(ti);
        if (tun >= 0) {
            where = String.format(Locale.ROOT, "in tunnel %d, %.0f m in", tun + 1,
                    net.distanceIntoTunnel(truth.position(ti)));
        } else {
            where = "open sky";
        }
        String text = String.format(Locale.ROOT, "%s   t = %.1f s   %.0f km/h   %s", car.car.id, state.time,
                truth.v(ti) * 3.6, where);
        g.setFill(Palette.withAlpha(Color.WHITE, 0.85));
        g.fillRoundRect(8, 8, text.length() * 6.6 + 16, 24, 6, 6);
        g.setFill(Palette.TEXT);
        g.fillText(text, 16, 13);

        // Scale bar.
        double meters = niceLength(120 / scale);
        double px = meters * scale;
        double y0 = h() - 20;
        g.setStroke(Palette.TEXT);
        g.setLineWidth(2);
        g.strokeLine(16, y0, 16 + px, y0);
        g.strokeLine(16, y0 - 4, 16, y0 + 4);
        g.strokeLine(16 + px, y0 - 4, 16 + px, y0 + 4);
        g.setTextBaseline(VPos.BOTTOM);
        g.setFill(Palette.TEXT);
        g.fillText(meters >= 1000 ? String.format(Locale.ROOT, "%.0f km", meters / 1000) : String.format(Locale.ROOT, "%.0f m", meters), 16, y0 - 6);
        g.setTextAlign(TextAlignment.RIGHT);
        g.setFill(Palette.MUTED);
        g.fillText(follow ? "following car (double-click for overview)" : "overview (double-click to follow)", w() - 12, h() - 10);
        g.setTextAlign(TextAlignment.LEFT);
        g.setTextBaseline(VPos.BASELINE);
    }

    private static double niceLength(double m) {
        double p = Math.pow(10, Math.floor(Math.log10(m)));
        double f = m / p;
        return (f < 2 ? 1 : f < 5 ? 2 : 5) * p;
    }
}
