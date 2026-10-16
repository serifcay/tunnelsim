package tunnelsim.app;

import javafx.animation.AnimationTimer;
import javafx.animation.PauseTransition;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.SimpleStringProperty;
import javafx.concurrent.Task;
import javafx.embed.swing.SwingFXUtils;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Separator;
import javafx.scene.control.Slider;
import javafx.scene.control.SplitPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToolBar;
import javafx.scene.control.Tooltip;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.stage.Stage;
import javafx.util.Duration;
import tunnelsim.engine.CarResult;
import tunnelsim.engine.CsvExport;
import tunnelsim.engine.RunEvent;
import tunnelsim.engine.RunResult;
import tunnelsim.engine.SimulationRun;
import tunnelsim.estimator.EstimatorFactory;
import tunnelsim.metrics.TrackMetrics;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioLoader;

import javax.imageio.ImageIO;
import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Function;

/**
 * The visual simulator. Pick a scenario, change parameters, and every change re-runs the whole
 * scenario in the background (deterministic for a given seed); the playhead stays where it was.
 *
 * <p>Snapshot mode for figures and headless checks:
 * {@code --snapshot=out.png --scenario=scenarios/s02-curved-exit.yaml --time=180 [--size=1600x1000] [--tab=Error]}
 */
public class Main extends Application {
    private static final Path SCENARIO_DIR = Path.of("scenarios");

    private final AppState state = new AppState();
    private ScenarioConfig base;
    private ScenarioConfig working;
    private Path scenarioFile;

    private MapView map;
    private StripView strip;
    private ChartsPane charts;
    private ParameterPanel params;
    private final ComboBox<Path> scenarioBox = new ComboBox<>();
    private final ComboBox<String> carBox = new ComboBox<>();
    private final Slider timeline = new Slider(0, 1, 0);
    private final Label timeLabel = new Label("0.0 s");
    private final Label status = new Label();
    private final TableView<TrackMetrics> metrics = new TableView<>();
    private final ListView<RunEvent> events = new ListView<>();
    private final Map<String, CheckBox> estimatorBoxes = new HashMap<>();
    private final ToggleButton play = new ToggleButton("▶ Play");
    private final ComboBox<String> speedBox = new ComboBox<>();
    private boolean updatingTimeline;
    private Task<RunResult> running;
    private boolean rerunPending;
    private final PauseTransition debounce = new PauseTransition(Duration.millis(250));
    private Stage stage;
    private final Map<String, String> args = new HashMap<>();

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) {
        this.stage = stage;
        for (String a : getParameters().getRaw()) {
            if (a.startsWith("--") && a.contains("=")) {
                args.put(a.substring(2, a.indexOf('=')), a.substring(a.indexOf('=') + 1));
            }
        }
        map = new MapView(state);
        strip = new StripView(state);
        charts = new ChartsPane(state, this::seek);
        params = new ParameterPanel(this::requestRun);
        debounce.setOnFinished(e -> runNow());

        BorderPane root = new BorderPane();
        root.setTop(toolbar());
        SplitPane left = new SplitPane(map, strip, charts);
        left.setOrientation(Orientation.VERTICAL);
        left.setDividerPositions(0.52, 0.68);
        SplitPane center = new SplitPane(left, sidePanel());
        center.setDividerPositions(0.74);
        root.setCenter(center);
        root.setBottom(timelineBar());

        String size = args.getOrDefault("size", "1500x950");
        String[] wh = size.split("x");
        Scene scene = new Scene(root, Double.parseDouble(wh[0]), Double.parseDouble(wh[1]));
        stage.setTitle("Through the Tunnel - positioning simulator");
        stage.setScene(scene);

        state.onChange(this::redrawAll);
        new AnimationTimer() {
            private long last;

            @Override
            public void handle(long now) {
                if (last != 0 && play.isSelected() && state.result != null) {
                    double dt = (now - last) / 1e9;
                    double speed = Double.parseDouble(speedBox.getValue().replace("x", ""));
                    double end = state.car().truth.duration();
                    double t = Math.min(end, state.time + dt * speed);
                    if (t >= end) {
                        play.setSelected(false);
                    }
                    seek(t);
                }
                last = now;
            }
        }.start();

        loadScenarioList();
        Path initial = args.containsKey("scenario") ? Path.of(args.get("scenario"))
                : scenarioBox.getItems().stream().filter(p -> p.getFileName().toString().startsWith("s02")).findFirst()
                .orElse(scenarioBox.getItems().isEmpty() ? null : scenarioBox.getItems().get(0));
        if (args.containsKey("snapshot")) {
            snapshotMode(stage, scene, initial);
            return;
        }
        stage.show();
        if (initial != null) {
            scenarioBox.setValue(initial);
        }
    }

    // ---- Layout ---------------------------------------------------------------------------

    private ToolBar toolbar() {
        scenarioBox.setPrefWidth(230);
        scenarioBox.setCellFactory(l -> pathCell());
        scenarioBox.setButtonCell(pathCell());
        scenarioBox.valueProperty().addListener((o, was, now) -> {
            if (now != null) {
                loadScenario(now);
            }
        });
        Button reload = new Button("Reset");
        reload.setTooltip(new Tooltip("Reload the scenario file, discarding parameter changes"));
        reload.setOnAction(e -> {
            if (scenarioFile != null) {
                loadScenario(scenarioFile);
            }
        });
        Button yaml = new Button("Edit YAML…");
        yaml.setOnAction(e -> editYaml());

        play.setOnAction(e -> {
            if (play.isSelected() && state.result != null && state.time >= state.car().truth.duration() - 0.05) {
                seek(0);
            }
            play.setText(play.isSelected() ? "⏸ Pause" : "▶ Play");
        });
        play.selectedProperty().addListener((o, was, now) -> play.setText(now ? "⏸ Pause" : "▶ Play"));
        Button restart = new Button("⏮");
        restart.setOnAction(e -> seek(0));
        speedBox.getItems().addAll("1x", "2x", "5x", "10x", "20x", "50x");
        speedBox.setValue("5x");

        carBox.setPrefWidth(110);
        carBox.valueProperty().addListener((o, was, now) -> {
            if (now != null && state.result != null) {
                state.car = Math.max(0, carBox.getItems().indexOf(now));
                resultViewsChanged();
            }
        });

        HBox estimators = new HBox(6);
        estimators.setAlignment(Pos.CENTER_LEFT);
        for (String id : EstimatorFactory.ALL) {
            CheckBox c = new CheckBox(id);
            c.setTextFill(Palette.estimator(id));
            c.setTooltip(new Tooltip(EstimatorFactory.describe(id)));
            c.setOnAction(e -> {
                if (c.isSelected()) {
                    state.visible.add(id);
                    if (working != null && !working.estimators.contains(id)) {
                        working.estimators.add(id);
                        requestRun();
                    }
                } else {
                    state.visible.remove(id);
                }
                sortVisible();
                resultViewsChanged();
            });
            estimatorBoxes.put(id, c);
            estimators.getChildren().add(c);
        }

        CheckBox follow = new CheckBox("Follow");
        follow.setSelected(true);
        follow.setOnAction(e -> map.setFollow(follow.isSelected()));
        CheckBox particles = new CheckBox("Particles");
        particles.setSelected(true);
        particles.setOnAction(e -> {
            state.showParticles = particles.isSelected();
            redrawAll();
        });
        CheckBox display = new CheckBox("Display marker");
        display.setTooltip(new Tooltip("Show the smoothed marker the driver would see (never moves backwards)"));
        display.setOnAction(e -> {
            state.showDisplayMarker = display.isSelected();
            redrawAll();
        });
        ComboBox<String> window = new ComboBox<>();
        window.getItems().addAll("±10 s", "±30 s", "±90 s", "full run");
        window.setValue("±30 s");
        window.valueProperty().addListener((o, was, now) -> {
            switch (now) {
                case "±10 s" -> charts.setWindow(false, 10);
                case "±90 s" -> charts.setWindow(false, 90);
                case "full run" -> charts.setWindow(true, 0);
                default -> charts.setWindow(false, 30);
            }
        });
        Button zoomIn = new Button("+");
        zoomIn.setOnAction(e -> map.zoom(1.5));
        Button zoomOut = new Button("−");
        zoomOut.setOnAction(e -> map.zoom(1 / 1.5));
        Button export = new Button("Export CSV");
        export.setOnAction(e -> exportCsv());

        return new ToolBar(new Label("Scenario"), scenarioBox, reload, yaml, new Separator(),
                play, restart, speedBox, new Separator(), new Label("Car"), carBox, new Separator(),
                estimators, new Separator(), follow, zoomOut, zoomIn, particles, display, new Separator(),
                new Label("Charts"), window, export);
    }

    private VBox sidePanel() {
        ScrollPane scroll = new ScrollPane(params);
        scroll.setFitToWidth(true);
        VBox.setVgrow(scroll, Priority.ALWAYS);

        metrics.setColumnResizePolicy(TableView.CONSTRAINED_RESIZE_POLICY_FLEX_LAST_COLUMN);
        metrics.setPrefHeight(180);
        metrics.setPlaceholder(new Label("no metrics"));
        metrics.getColumns().add(col("est", m -> m.estimator()));
        metrics.getColumns().add(col("rms", m -> f(m.rmsTunnel())));
        metrics.getColumns().add(col("max", m -> f(m.maxTunnel())));
        metrics.getColumns().add(col("exit", m -> f(m.maxExitError())));
        metrics.getColumns().add(col("call s", m -> f(m.worstCallout())));
        metrics.getColumns().add(col("late", m -> Integer.toString(m.lateCallouts())));
        metrics.getColumns().add(col("branch s", m -> f(m.branchDecision())));
        metrics.getColumns().add(col("in 2σ", m -> Double.isNaN(m.within2Sigma()) ? "-" : String.format(Locale.ROOT, "%.0f%%", 100 * m.within2Sigma())));
        metrics.setTooltip(new Tooltip("""
                rms/max: along-road error inside tunnels [m]
                exit: error when the car leaves a tunnel [m]
                call s: worst callout timing error [s]; late: callouts that fired late or not at all
                branch s: time after a fork until P(true branch) >= 0.9
                in 2σ: share of tunnel time the error was inside the claimed 2-sigma (95% is ideal)"""));

        events.setPrefHeight(220);
        events.setCellFactory(l -> new ListCell<>() {
            @Override
            protected void updateItem(RunEvent e, boolean empty) {
                super.updateItem(e, empty);
                if (empty || e == null) {
                    setText(null);
                    setTextFill(Palette.TEXT);
                } else {
                    setText(String.format(Locale.ROOT, "%6.1f s  %s%s", e.t(), e.estimator() == null ? "" : "[" + e.estimator() + "] ", e.text()));
                    setTextFill(e.estimator() == null ? Palette.TEXT : Palette.estimator(e.estimator()));
                    setFont(Font.font("Menlo", 11));
                }
            }
        });
        events.getSelectionModel().selectedItemProperty().addListener((o, was, now) -> {
            if (now != null) {
                seek(Math.max(0, now.t() - 2));
            }
        });
        Label m = new Label("Metrics (selected car)");
        Label ev = new Label("Events (click to jump)");
        m.setFont(Font.font("System", 12));
        VBox box = new VBox(4, scroll, m, metrics, ev, events);
        box.setPadding(new Insets(4));
        box.setMinWidth(330);
        return box;
    }

    private HBox timelineBar() {
        timeline.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(timeline, Priority.ALWAYS);
        timeline.valueProperty().addListener((o, was, now) -> {
            if (!updatingTimeline) {
                seek(now.doubleValue());
            }
        });
        timeLabel.setMinWidth(70);
        timeLabel.setFont(Font.font("Menlo", 12));
        status.setMinWidth(260);
        status.setTextFill(Palette.MUTED);
        HBox bar = new HBox(10, timeLabel, timeline, status);
        bar.setAlignment(Pos.CENTER_LEFT);
        bar.setPadding(new Insets(6, 10, 6, 10));
        return bar;
    }

    private static TableColumn<TrackMetrics, String> col(String name, Function<TrackMetrics, String> f) {
        TableColumn<TrackMetrics, String> c = new TableColumn<>(name);
        c.setCellValueFactory(d -> new SimpleStringProperty(f.apply(d.getValue())));
        c.setSortable(false);
        return c;
    }

    private ListCell<Path> pathCell() {
        return new ListCell<>() {
            @Override
            protected void updateItem(Path p, boolean empty) {
                super.updateItem(p, empty);
                setText(empty || p == null ? null : p.getFileName().toString().replace(".yaml", ""));
            }
        };
    }

    // ---- Scenario handling ----------------------------------------------------------------

    private void loadScenarioList() {
        scenarioBox.getItems().setAll(ScenarioLoader.list(SCENARIO_DIR));
    }

    private void loadScenario(Path file) {
        if (args.containsKey("snapshot")) {
            return;
        }
        try {
            scenarioFile = file;
            base = ScenarioLoader.load(file);
            setWorking(ScenarioLoader.copy(base), true);
        } catch (RuntimeException ex) {
            error("Cannot load scenario", ex);
        }
    }

    private void setWorking(ScenarioConfig cfg, boolean resetVisible) {
        working = cfg;
        if (resetVisible) {
            state.visible.clear();
            state.visible.addAll(cfg.estimators);
            state.time = 0;
            state.car = 0;
        }
        estimatorBoxes.forEach((id, box) -> box.setSelected(state.visible.contains(id)));
        sortVisible();
        params.load(working);
        runNow();
    }

    private void sortVisible() {
        List<String> sorted = new ArrayList<>(EstimatorFactory.ALL);
        sorted.retainAll(state.visible);
        state.visible.clear();
        state.visible.addAll(sorted);
    }

    private void requestRun() {
        debounce.playFromStart();
    }

    private void runNow() {
        if (working == null) {
            return;
        }
        if (running != null && running.isRunning()) {
            rerunPending = true;
            return;
        }
        ScenarioConfig cfg;
        try {
            cfg = ScenarioLoader.validate(ScenarioLoader.copy(working));
        } catch (RuntimeException ex) {
            error("Invalid scenario", ex);
            return;
        }
        status.setText("running " + cfg.name + " …");
        Task<RunResult> task = new Task<>() {
            @Override
            protected RunResult call() {
                return SimulationRun.run(cfg);
            }
        };
        task.setOnSucceeded(e -> {
            setResult(task.getValue());
            if (rerunPending) {
                rerunPending = false;
                runNow();
            }
        });
        task.setOnFailed(e -> {
            error("Simulation failed", task.getException());
            rerunPending = false;
        });
        running = task;
        Thread th = new Thread(task, "simulation");
        th.setDaemon(true);
        th.start();
    }

    private void setResult(RunResult r) {
        state.result = r;
        List<String> cars = r.cars().stream().map(c -> c.car.id).toList();
        String selected = carBox.getValue();
        carBox.getItems().setAll(cars);
        state.car = cars.contains(selected) ? cars.indexOf(selected) : Math.min(state.car, cars.size() - 1);
        carBox.setValue(cars.get(state.car));
        status.setText(String.format(Locale.ROOT, "%s, seed %d: computed in %.2f s", r.scenario().name,
                r.scenario().seed, r.computeSeconds()));
        resultViewsChanged();
    }

    private void resultViewsChanged() {
        CarResult car = state.car();
        if (car == null) {
            return;
        }
        double end = car.truth.duration();
        state.time = Math.min(state.time, end);
        updatingTimeline = true;
        timeline.setMax(end);
        timeline.setValue(state.time);
        updatingTimeline = false;
        map.resultChanged();
        charts.resultChanged();
        metrics.getItems().setAll(car.metrics.values().stream().filter(m -> state.visible.contains(m.estimator())).toList());
        events.getItems().setAll(car.events.stream()
                .filter(e -> e.estimator() == null || state.visible.contains(e.estimator())).toList());
        redrawAll();
    }

    private void seek(double t) {
        if (state.result == null) {
            return;
        }
        state.time = Math.max(0, Math.min(t, state.car().truth.duration()));
        updatingTimeline = true;
        timeline.setValue(state.time);
        updatingTimeline = false;
        state.changed();
    }

    private void redrawAll() {
        timeLabel.setText(String.format(Locale.ROOT, "%.1f s", state.time));
        map.redraw();
        strip.redraw();
        charts.refreshCursor();
    }

    private void editYaml() {
        if (working == null) {
            return;
        }
        Dialog<ButtonType> d = new Dialog<>();
        d.setTitle("Scenario YAML");
        d.setHeaderText("Edit any parameter; Apply re-runs the scenario. Units are SI (m, s, m/s, rad).");
        TextArea text = new TextArea(ScenarioLoader.toYaml(working));
        text.setFont(Font.font("Menlo", 12));
        text.setPrefSize(720, 640);
        d.getDialogPane().setContent(text);
        ButtonType apply = new ButtonType("Apply", javafx.scene.control.ButtonBar.ButtonData.OK_DONE);
        d.getDialogPane().getButtonTypes().addAll(apply, ButtonType.CANCEL);
        d.initOwner(stage);
        d.showAndWait().ifPresent(b -> {
            if (b == apply) {
                try {
                    setWorking(ScenarioLoader.parse(text.getText()), false);
                } catch (RuntimeException ex) {
                    error("Invalid YAML", ex);
                }
            }
        });
    }

    private void exportCsv() {
        if (state.result == null) {
            return;
        }
        try {
            Path dir = CsvExport.write(state.result, Path.of("results", "export"));
            status.setText("exported to " + dir);
        } catch (RuntimeException ex) {
            error("Export failed", ex);
        }
    }

    private void error(String title, Throwable ex) {
        Throwable cause = ex;
        while (cause.getCause() != null && cause.getMessage() == null) {
            cause = cause.getCause();
        }
        status.setText(title);
        if (args.containsKey("snapshot")) {
            ex.printStackTrace();
            return;
        }
        Alert a = new Alert(Alert.AlertType.ERROR, String.valueOf(cause.getMessage()), ButtonType.OK);
        a.setHeaderText(title);
        a.initOwner(stage);
        a.show();
    }

    private static String f(double v) {
        if (Double.isNaN(v)) {
            return "-";
        }
        return Math.abs(v) >= 100 ? String.format(Locale.ROOT, "%.0f", v) : String.format(Locale.ROOT, "%.1f", v);
    }

    // ---- Snapshot mode --------------------------------------------------------------------

    private void snapshotMode(Stage stage, Scene scene, Path scenario) {
        try {
            ScenarioConfig cfg = ScenarioLoader.load(scenario);
            scenarioFile = scenario;
            scenarioBox.getItems().stream().filter(p -> p.getFileName().equals(scenario.getFileName())).findFirst()
                    .ifPresent(scenarioBox::setValue);
            base = cfg;
            working = ScenarioLoader.copy(cfg);
            state.visible.addAll(working.estimators);
            sortVisible();
            estimatorBoxes.forEach((id, box) -> box.setSelected(state.visible.contains(id)));
            params.load(working);
            RunResult r = SimulationRun.run(ScenarioLoader.copy(working));
            stage.show();
            setResult(r);
            seek(Double.parseDouble(args.getOrDefault("time", "0")));
            String tab = args.get("tab");
            if (tab != null) {
                charts.getTabs().stream().filter(t -> t.getText().equalsIgnoreCase(tab)).findFirst()
                        .ifPresent(t -> charts.getSelectionModel().select(t));
            }
            if (args.containsKey("overview")) {
                map.setFollow(false);
            }
            PauseTransition wait = new PauseTransition(Duration.millis(600));
            wait.setOnFinished(e -> {
                redrawAll();
                WritableImage img = scene.snapshot(null);
                try {
                    ImageIO.write(SwingFXUtils.fromFXImage(img, null), "png", new File(args.get("snapshot")));
                    System.out.println("snapshot written to " + args.get("snapshot"));
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
                Platform.exit();
            });
            wait.play();
        } catch (RuntimeException ex) {
            ex.printStackTrace();
            Platform.exit();
        }
    }
}
