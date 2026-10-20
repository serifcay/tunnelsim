package tunnelsim.app;

import javafx.geometry.Insets;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioConfig.CarConfig;
import tunnelsim.scenario.ScenarioConfig.EventConfig;
import tunnelsim.scenario.ScenarioConfig.SegmentConfig;
import tunnelsim.scenario.ScenarioConfig.TunnelConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

/**
 * Controls for the parameters an interviewer is most likely to ask about. Every control writes
 * into the working scenario and triggers a re-run; anything else can be changed in the YAML editor.
 */
final class ParameterPanel extends VBox {
    private final Runnable onChange;
    private ScenarioConfig cfg;
    private boolean loading;
    private final List<Runnable> loaders = new ArrayList<>();
    // Base IMU values for the quality multiplier.
    // The scenario's own IMU noise and bias values; the "IMU quality" slider multiplies them.
    private double baseAccNoise, baseAccBias, baseGyroNoise, baseGyroBias;

    ParameterPanel(Runnable onChange) {
        this.onChange = onChange;
        setSpacing(6);
        setPadding(new Insets(6));
        getChildren().addAll(scenarioSection(), drivingSection(), sensorSection(), estimatorSection());
    }

    /** Shows the values of a (new) working scenario. */
    void load(ScenarioConfig config) {
        this.cfg = config;
        baseAccNoise = config.sensors.accel.noise;
        baseAccBias = config.sensors.accel.biasSigma;
        baseGyroNoise = config.sensors.gyro.noise;
        baseGyroBias = config.sensors.gyro.biasSigma;
        loading = true;
        loaders.forEach(Runnable::run);
        loading = false;
    }

    /** A control changed the scenario: re-run (ignored while load() fills in the controls). */
    private void changed() {
        if (!loading && cfg != null) {
            onChange.run();
        }
    }

    // ---- Sections -------------------------------------------------------------------------

    /** Seed, device profile, day/night and option E (BLE, OBD). */
    private TitledPane scenarioSection() {
        GridPane g = grid();
        int r = 0;
        TextField seed = new TextField();
        seed.setPrefColumnCount(8);
        seed.setOnAction(e -> {
            try {
                cfg.seed = Long.parseLong(seed.getText().trim());
                changed();
            } catch (NumberFormatException ignored) {
                seed.setText(Long.toString(cfg.seed));
            }
        });
        Button next = new Button("New seed");
        next.setOnAction(e -> {
            cfg.seed++;
            seed.setText(Long.toString(cfg.seed));
            changed();
        });
        loaders.add(() -> seed.setText(Long.toString(cfg.seed)));
        g.addRow(r++, new Label("Seed"), seed, next);

        ComboBox<String> device = combo(List.of("GENERIC", "IOS", "LOW_END"), () -> cfg.device, v -> cfg.device = v);
        g.addRow(r++, new Label("Device"), device);
        ComboBox<String> light = combo(List.of("day", "night"), () -> cfg.road.lighting, v -> cfg.road.lighting = v);
        g.addRow(r++, new Label("Lighting"), light);
        CheckBox ble = check("BLE beacons", () -> cfg.options.ble, v -> {
            cfg.options.ble = v;
            ensureEstimator("D+E", v || cfg.options.obd);
        });
        CheckBox obd = check("OBD speed", () -> cfg.options.obd, v -> {
            cfg.options.obd = v;
            ensureEstimator("D+E", v || cfg.options.ble);
        });
        g.add(new Label("Option E"), 0, r);
        g.add(new FlowPane(8, 4, ble, obd), 1, r++, 2, 1);
        return section("Scenario", g);
    }

    /** Tunnel length, driver speed, route following, and buttons that add traffic events or a phone shift. */
    private TitledPane drivingSection() {
        GridPane g = grid();
        int r = 0;
        r = slider(g, r, "Tunnel 1 length", "%.0f m", 500, 5000, () -> firstTunnel() == null ? 0 : firstTunnel().to - firstTunnel().from,
                this::setTunnelLength);
        r = slider(g, r, "Speed factor", "%.2f", 0.4, 1.3, () -> firstCar().driver.speedFactor, v -> firstCar().driver.speedFactor = v);
        CheckBox follow = check("Driver follows route at forks", () -> firstCar().driver.pFollowRoute > 0.5,
                v -> firstCar().driver.pFollowRoute = v ? 1 : 0);
        g.add(follow, 0, r++, 3, 1);

        Button stop = new Button("+ Stop 60 s");
        stop.setOnAction(e -> addEvent("stop", ev -> ev.duration = 60));
        Button jam = new Button("+ Jam");
        jam.setOnAction(e -> addEvent("stopAndGo", ev -> {
            ev.length = 1000;
            ev.speed = 10;
            ev.stopTime = 8;
            ev.goTime = 20;
        }));
        Button brake = new Button("+ Hard brake");
        brake.setOnAction(e -> addEvent("hardBrake", ev -> {
            ev.speed = 5;
            ev.decel = 6;
        }));
        Button shift = new Button("+ Phone shift");
        shift.setOnAction(e -> addEvent("mountShift", ev -> {
            ev.yawDeg = 25;
            ev.pitchDeg = 10;
        }));
        Button clear = new Button("Clear events");
        clear.setOnAction(e -> {
            firstCar().events.clear();
            changed();
        });
        Label hint = new Label("Events are placed in the middle of tunnel 1.");
        hint.setFont(Font.font(10));
        g.add(new FlowPane(4, 4, stop, jam, brake, shift, clear), 0, r++, 3, 1);
        g.add(hint, 0, r, 3, 1);
        return section("Tunnel and driving (car 1)", g);
    }

    /** IMU quality, GNSS reacquisition delay, barometer on/off, BLE beacon spacing and failures. */
    private TitledPane sensorSection() {
        GridPane g = grid();
        int r = 0;
        r = slider(g, r, "IMU quality", "x%.1f", 0.25, 4, () -> baseAccNoise == 0 ? 1 : cfg.sensors.accel.noise / baseAccNoise, v -> {
            cfg.sensors.accel.noise = baseAccNoise * v;
            cfg.sensors.accel.biasSigma = baseAccBias * v;
            cfg.sensors.gyro.noise = baseGyroNoise * v;
            cfg.sensors.gyro.biasSigma = baseGyroBias * v;
        });
        r = slider(g, r, "GNSS reacquire max", "%.0f s", 1, 20, () -> cfg.sensors.gnss.reacquireDelay[1],
                v -> cfg.sensors.gnss.reacquireDelay = new double[]{Math.min(1, v), v});
        CheckBox baro = check("Barometer present", () -> cfg.sensors.baro.enabled, v -> cfg.sensors.baro.enabled = v);
        g.add(baro, 0, r++, 3, 1);
        r = slider(g, r, "BLE spacing", "%.0f m", 25, 400, () -> cfg.sensors.ble.spacing, v -> cfg.sensors.ble.spacing = v);
        r = slider(g, r, "BLE failed", "%.0f %%", 0, 80, () -> cfg.sensors.ble.failedFraction * 100,
                v -> cfg.sensors.ble.failedFraction = v / 100);
        return section("Phone sensors", g);
    }

    /** Particle count, alignment mode, callout policy and an on/off switch for each aiding source. */
    private TitledPane estimatorSection() {
        GridPane g = grid();
        int r = 0;
        r = slider(g, r, "Particles", "%.0f", 100, 5000, () -> cfg.estimator.particles, v -> cfg.estimator.particles = (int) Math.round(v));
        ComboBox<String> align = combo(List.of("ESTIMATED", "TRUTH"), () -> cfg.estimator.alignment, v -> cfg.estimator.alignment = v);
        g.addRow(r++, new Label("Alignment"), align);
        ComboBox<String> policy = combo(List.of("CONSERVATIVE", "MEAN"), () -> cfg.nav.calloutPolicy, v -> cfg.nav.calloutPolicy = v);
        g.addRow(r++, new Label("Callouts"), policy);
        FlowPane aids = new FlowPane(8, 4,
                check("curvature", () -> cfg.estimator.aiding.curvature, v -> cfg.estimator.aiding.curvature = v),
                check("lateral acc", () -> cfg.estimator.aiding.lateralAccel, v -> cfg.estimator.aiding.lateralAccel = v),
                check("baro", () -> cfg.estimator.aiding.baro, v -> cfg.estimator.aiding.baro = v),
                check("stops (ZUPT)", () -> cfg.estimator.aiding.zupt, v -> cfg.estimator.aiding.zupt = v),
                check("speed prior", () -> cfg.estimator.aiding.speedPrior, v -> cfg.estimator.aiding.speedPrior = v),
                check("portal (light)", () -> cfg.estimator.aiding.portal, v -> cfg.estimator.aiding.portal = v),
                check("cell", () -> cfg.estimator.aiding.cell, v -> cfg.estimator.aiding.cell = v));
        g.add(new Label("Aiding"), 0, r);
        g.add(aids, 1, r, 2, 1);
        return section("Estimators", g);
    }

    // ---- Helpers --------------------------------------------------------------------------

    private TunnelConfig firstTunnel() {
        return cfg.road.tunnels.isEmpty() ? null : cfg.road.tunnels.get(0);
    }

    private CarConfig firstCar() {
        return cfg.cars.get(0);
    }

    /** Sets tunnel 1's length; lengthens the road if needed and moves later tunnels so nothing overlaps. */
    private void setTunnelLength(double length) {
        TunnelConfig t = firstTunnel();
        if (t == null) {
            return;
        }
        t.to = t.from + length;
        double mainLength = cfg.road.main.stream().mapToDouble(s -> s.length).sum();
        double needed = t.to + 600 - mainLength;
        if (needed > 0) {
            SegmentConfig last = cfg.road.main.get(cfg.road.main.size() - 1);
            last.length += needed;
        }
        // Keep later tunnels and forks consistent.
        for (int i = 1; i < cfg.road.tunnels.size(); i++) {
            TunnelConfig o = cfg.road.tunnels.get(i);
            if (o.from < t.to + 100) {
                double len = o.to - o.from;
                o.from = t.to + 150;
                o.to = o.from + len;
            }
        }
        double newMain = cfg.road.main.stream().mapToDouble(s -> s.length).sum();
        for (TunnelConfig o : cfg.road.tunnels) {
            if (o.to > newMain - 100) {
                cfg.road.main.get(cfg.road.main.size() - 1).length += o.to - newMain + 300;
                newMain = cfg.road.main.stream().mapToDouble(s -> s.length).sum();
            }
        }
    }

    /** Adds a scripted event for car 1 in the middle of tunnel 1. */
    private void addEvent(String type, Consumer<EventConfig> setup) {
        TunnelConfig t = firstTunnel();
        EventConfig e = new EventConfig();
        e.type = type;
        e.at = t == null ? 1500 : (t.from + t.to) / 2;
        setup.accept(e);
        firstCar().events.add(e);
        changed();
    }

    /** Makes sure an estimator is computed (e.g. D+E once BLE or OBD is switched on). */
    private void ensureEstimator(String id, boolean present) {
        if (present && !cfg.estimators.contains(id)) {
            cfg.estimators.add(id);
        }
    }

    private static GridPane grid() {
        GridPane g = new GridPane();
        g.setHgap(6);
        g.setVgap(5);
        return g;
    }

    private static TitledPane section(String title, GridPane content) {
        TitledPane p = new TitledPane(title, content);
        p.setAnimated(false);
        return p;
    }

    /** Adds a labelled slider bound to one scenario value; it applies when the user releases it. */
    private int slider(GridPane g, int row, String name, String format, double min, double max,
                       java.util.function.DoubleSupplier get, DoubleConsumer set) {
        Slider s = new Slider(min, max, min);
        Label value = new Label();
        value.setMinWidth(62);
        s.setPrefWidth(130);
        GridPane.setHgrow(s, Priority.ALWAYS);
        // Show the value live while dragging, but only apply it (and re-run) once the drag ends or after a click
        // on the track.
        s.valueProperty().addListener(o -> value.setText(String.format(Locale.ROOT, format, s.getValue())));
        s.valueChangingProperty().addListener((o, was, now) -> {
            if (!now) {
                set.accept(s.getValue());
                changed();
            }
        });
        s.setOnMouseReleased(e -> {
            if (!s.isValueChanging()) {
                set.accept(s.getValue());
                changed();
            }
        });
        // When a scenario is loaded, move the slider to its value.
        loaders.add(() -> {
            s.setValue(Math.max(min, Math.min(max, get.getAsDouble())));
            value.setText(String.format(Locale.ROOT, format, s.getValue()));
        });
        g.addRow(row, new Label(name), s, value);
        return row + 1;
    }

    /** A checkbox bound to one scenario switch. */
    private CheckBox check(String name, java.util.function.BooleanSupplier get, Consumer<Boolean> set) {
        CheckBox c = new CheckBox(name);
        c.selectedProperty().addListener((o, was, now) -> {
            if (!loading) {
                set.accept(now);
                changed();
            }
        });
        loaders.add(() -> c.setSelected(get.getAsBoolean()));
        return c;
    }

    /** A drop-down bound to one scenario setting. */
    private ComboBox<String> combo(List<String> values, java.util.function.Supplier<String> get, Consumer<String> set) {
        ComboBox<String> c = new ComboBox<>();
        c.getItems().addAll(values);
        c.valueProperty().addListener((o, was, now) -> {
            if (!loading && now != null) {
                set.accept(now);
                changed();
            }
        });
        loaders.add(() -> c.setValue(get.get()));
        return c;
    }
}
