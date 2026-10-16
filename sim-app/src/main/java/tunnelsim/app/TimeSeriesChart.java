package tunnelsim.app;

import javafx.geometry.VPos;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.TextAlignment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleConsumer;

/**
 * A light canvas time-series plot. Draws per-pixel min/max so 100 Hz series stay fast, shades
 * tunnel intervals, shows the playhead, and follows it with a moving window or shows the full run.
 */
final class TimeSeriesChart extends CanvasPane {
    record Series(String name, Color color, double[] t, double[] y, double[] lower, double[] upper, boolean dashed) {
        static Series line(String name, Color color, double[] t, double[] y) {
            return new Series(name, color, t, y, null, null, false);
        }
    }

    private static final double LEFT = 58, RIGHT = 12, TOP = 22, BOTTOM = 22;

    private final String title;
    private final String unit;
    private final List<Series> series = new ArrayList<>();
    private List<double[]> shaded = List.of();
    private double cursor;
    private double halfWindow = 30;
    private boolean full;
    private boolean log;
    private Double fixedMin, fixedMax;
    private double runEnd = 1;
    private DoubleConsumer onSeek;

    TimeSeriesChart(String title, String unit) {
        this.title = title;
        this.unit = unit;
        setOnMouseClicked(e -> {
            if (onSeek != null && e.getX() > LEFT) {
                double[] r = timeRange();
                onSeek.accept(r[0] + (e.getX() - LEFT) / (w() - LEFT - RIGHT) * (r[1] - r[0]));
            }
        });
    }

    void setSeries(List<Series> s, List<double[]> shadedIntervals, double end) {
        series.clear();
        series.addAll(s);
        shaded = shadedIntervals;
        runEnd = Math.max(1, end);
        redraw();
    }

    void setCursor(double t) {
        cursor = t;
        redraw();
    }

    void setWindow(boolean fullRun, double half) {
        full = fullRun;
        halfWindow = half;
        redraw();
    }

    void setLog(boolean logScale) {
        log = logScale;
    }

    void setRange(Double min, Double max) {
        fixedMin = min;
        fixedMax = max;
    }

    void onSeek(DoubleConsumer c) {
        onSeek = c;
    }

    private double[] timeRange() {
        if (full) {
            return new double[]{0, runEnd};
        }
        return new double[]{cursor - halfWindow, cursor + halfWindow};
    }

    @Override
    void redraw() {
        GraphicsContext g = g();
        g.setFill(Palette.PANEL);
        g.fillRect(0, 0, w(), h());
        double pw = w() - LEFT - RIGHT, ph = h() - TOP - BOTTOM;
        if (pw < 10 || ph < 10) {
            return;
        }
        double[] tr = timeRange();
        double t0 = tr[0], t1 = tr[1];

        // Y range over visible data.
        double ymin = Double.MAX_VALUE, ymax = -Double.MAX_VALUE;
        for (Series s : series) {
            int a = lowerIndex(s.t(), t0), b = lowerIndex(s.t(), t1) + 1;
            for (int i = Math.max(0, a); i < Math.min(s.t().length, b); i++) {
                double v = val(s.y()[i]);
                if (Double.isFinite(v)) {
                    ymin = Math.min(ymin, v);
                    ymax = Math.max(ymax, v);
                }
            }
        }
        if (fixedMin != null) {
            ymin = val(fixedMin);
        }
        if (fixedMax != null) {
            ymax = val(fixedMax);
        }
        if (ymin == Double.MAX_VALUE) {
            ymin = 0;
            ymax = 1;
        }
        if (ymax - ymin < 1e-9) {
            ymax = ymin + 1;
        }
        double pad = (ymax - ymin) * 0.08;
        ymin -= pad;
        ymax += pad;
        final double yLo = ymin, yHi = ymax;
        java.util.function.DoubleUnaryOperator X = t -> LEFT + (t - t0) / (t1 - t0) * pw;
        java.util.function.DoubleUnaryOperator Y = v -> TOP + (1 - (val(v) - yLo) / (yHi - yLo)) * ph;

        // Shaded tunnel intervals.
        g.setFill(Color.web("#eceae5"));
        for (double[] iv : shaded) {
            double a = Math.max(LEFT, X.applyAsDouble(iv[0])), b = Math.min(LEFT + pw, X.applyAsDouble(iv[1]));
            if (b > a) {
                g.fillRect(a, TOP, b - a, ph);
            }
        }
        // Grid.
        g.setFont(Font.font("System", 10));
        g.setStroke(Palette.GRID);
        g.setLineWidth(1);
        g.setFill(Palette.MUTED);
        g.setTextAlign(TextAlignment.RIGHT);
        g.setTextBaseline(VPos.CENTER);
        double ystep = niceStep((yHi - yLo) / 4);
        for (double v = Math.ceil(yLo / ystep) * ystep; v <= yHi; v += ystep) {
            double y = TOP + (1 - (v - yLo) / (yHi - yLo)) * ph;
            g.strokeLine(LEFT, y, LEFT + pw, y);
            g.fillText(fmt(log ? Math.pow(10, v) : v), LEFT - 4, y);
        }
        g.setTextAlign(TextAlignment.CENTER);
        g.setTextBaseline(VPos.TOP);
        double tstep = niceStep((t1 - t0) / 6);
        for (double t = Math.ceil(t0 / tstep) * tstep; t <= t1; t += tstep) {
            double x = X.applyAsDouble(t);
            g.strokeLine(x, TOP, x, TOP + ph);
            g.fillText(String.format(Locale.ROOT, "%.0f s", t), x, TOP + ph + 4);
        }

        g.save();
        g.beginPath();
        g.rect(LEFT, TOP, pw, ph);
        g.clip();
        // Bands first.
        for (Series s : series) {
            if (s.lower() == null) {
                continue;
            }
            int a = Math.max(0, lowerIndex(s.t(), t0)), b = Math.min(s.t().length, lowerIndex(s.t(), t1) + 2);
            int stride = Math.max(1, (b - a) / (int) Math.max(1, pw));
            List<double[]> upper = new ArrayList<>(), lower = new ArrayList<>();
            for (int i = a; i < b; i += stride) {
                if (Double.isFinite(s.lower()[i]) && Double.isFinite(s.upper()[i])) {
                    upper.add(new double[]{X.applyAsDouble(s.t()[i]), Y.applyAsDouble(s.upper()[i])});
                    lower.add(new double[]{X.applyAsDouble(s.t()[i]), Y.applyAsDouble(s.lower()[i])});
                }
            }
            if (upper.size() > 1) {
                double[] xs = new double[upper.size() * 2], ys = new double[upper.size() * 2];
                for (int k = 0; k < upper.size(); k++) {
                    xs[k] = upper.get(k)[0];
                    ys[k] = upper.get(k)[1];
                    xs[xs.length - 1 - k] = lower.get(k)[0];
                    ys[ys.length - 1 - k] = lower.get(k)[1];
                }
                g.setFill(Palette.withAlpha(s.color(), 0.15));
                g.fillPolygon(xs, ys, xs.length);
            }
        }
        for (Series s : series) {
            drawSeries(g, s, t0, t1, X, Y, pw);
        }
        g.restore();

        // Playhead.
        double cx = X.applyAsDouble(cursor);
        g.setStroke(Palette.TEXT);
        g.setLineWidth(1);
        g.strokeLine(cx, TOP, cx, TOP + ph);

        // Title and legend.
        g.setTextAlign(TextAlignment.LEFT);
        g.setTextBaseline(VPos.CENTER);
        g.setFont(Font.font("System", 11));
        g.setFill(Palette.TEXT);
        g.fillText(title + (unit.isEmpty() ? "" : " [" + unit + "]"), LEFT, 10);
        double lx = LEFT + 8 + title.length() * 7 + unit.length() * 7 + 20;
        for (Series s : series) {
            g.setFill(s.color());
            g.fillRect(lx, 7, 12, 4);
            g.setFill(Palette.TEXT);
            g.fillText(s.name(), lx + 16, 10);
            lx += 26 + s.name().length() * 6.5;
        }
        g.setTextBaseline(VPos.BASELINE);
    }

    private void drawSeries(GraphicsContext g, Series s, double t0, double t1,
                            java.util.function.DoubleUnaryOperator X, java.util.function.DoubleUnaryOperator Y, double pw) {
        int a = Math.max(0, lowerIndex(s.t(), t0) - 1), b = Math.min(s.t().length, lowerIndex(s.t(), t1) + 2);
        if (b - a < 1) {
            return;
        }
        g.setStroke(s.color());
        g.setLineWidth(1.3);
        if (s.dashed()) {
            g.setLineDashes(5, 4);
        }
        g.beginPath();
        boolean pen = false;
        int col = Integer.MIN_VALUE;
        double cmin = 0, cmax = 0, last = 0;
        for (int i = a; i < b; i++) {
            double v = s.y()[i];
            if (!Double.isFinite(v)) {
                if (col != Integer.MIN_VALUE) {
                    flush(g, col, cmin, cmax, last, pen);
                }
                col = Integer.MIN_VALUE;
                pen = false;
                continue;
            }
            int c = (int) X.applyAsDouble(s.t()[i]);
            double y = Y.applyAsDouble(v);
            if (c != col) {
                if (col != Integer.MIN_VALUE) {
                    pen = flush(g, col, cmin, cmax, last, pen);
                }
                col = c;
                cmin = cmax = y;
            } else {
                cmin = Math.min(cmin, y);
                cmax = Math.max(cmax, y);
            }
            last = y;
        }
        if (col != Integer.MIN_VALUE) {
            flush(g, col, cmin, cmax, last, pen);
        }
        g.stroke();
        g.setLineDashes();
    }

    private static boolean flush(GraphicsContext g, int col, double cmin, double cmax, double last, boolean pen) {
        if (!pen) {
            g.moveTo(col, cmin);
        } else {
            g.lineTo(col, cmin);
        }
        g.lineTo(col, cmax);
        g.lineTo(col, last);
        return true;
    }

    private double val(double v) {
        return log ? Math.log10(Math.max(1e-3, v)) : v;
    }

    private static int lowerIndex(double[] t, double x) {
        int i = Arrays.binarySearch(t, x);
        return i >= 0 ? i : -i - 2;
    }

    private static double niceStep(double m) {
        if (m <= 0 || !Double.isFinite(m)) {
            return 1;
        }
        double p = Math.pow(10, Math.floor(Math.log10(m)));
        double f = m / p;
        return (f < 2 ? 1 : f < 5 ? 2 : 5) * p;
    }

    private static String fmt(double v) {
        double a = Math.abs(v);
        if (a >= 1000) {
            return String.format(Locale.ROOT, "%.0f", v);
        }
        if (a >= 10) {
            return String.format(Locale.ROOT, "%.0f", v);
        }
        if (a >= 1) {
            return String.format(Locale.ROOT, "%.1f", v);
        }
        return String.format(Locale.ROOT, "%.2f", v);
    }
}
