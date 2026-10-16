package tunnelsim.scenario;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public final class ScenarioLoader {
    private static final ObjectMapper YAML = new ObjectMapper(new YAMLFactory())
            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, true)
            .configure(DeserializationFeature.ACCEPT_SINGLE_VALUE_AS_ARRAY, true);

    private ScenarioLoader() {
    }

    public static ScenarioConfig load(Path file) {
        try {
            return validate(YAML.readValue(file.toFile(), ScenarioConfig.class));
        } catch (IOException e) {
            throw new UncheckedIOException("Cannot read scenario " + file + ": " + e.getMessage(), e);
        }
    }

    public static ScenarioConfig load(InputStream in) {
        try {
            return validate(YAML.readValue(in, ScenarioConfig.class));
        } catch (IOException e) {
            throw new UncheckedIOException("Cannot read scenario: " + e.getMessage(), e);
        }
    }

    public static ScenarioConfig parse(String yaml) {
        try {
            return validate(YAML.readValue(yaml, ScenarioConfig.class));
        } catch (IOException e) {
            throw new UncheckedIOException("Cannot parse scenario: " + e.getMessage(), e);
        }
    }

    public static String toYaml(ScenarioConfig config) {
        try {
            return YAML.writeValueAsString(config);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /** Deep copy, so a UI or a parameter sweep can modify a scenario without touching the original. */
    public static ScenarioConfig copy(ScenarioConfig config) {
        return YAML.convertValue(config, ScenarioConfig.class);
    }

    /** All *.yaml files in a directory, sorted by file name. */
    public static List<Path> list(Path dir) {
        if (!Files.isDirectory(dir)) {
            return List.of();
        }
        try (Stream<Path> files = Files.list(dir)) {
            return new ArrayList<>(files.filter(p -> p.toString().endsWith(".yaml")).sorted().toList());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public static ScenarioConfig validate(ScenarioConfig c) {
        List<String> errors = new ArrayList<>();
        if (c.road.main.isEmpty()) {
            errors.add("road.main must contain at least one segment");
        }
        double mainLength = c.road.main.stream().mapToDouble(s -> s.length).sum();
        for (ScenarioConfig.SegmentConfig s : c.road.main) {
            checkSegment(s, "road.main", errors);
        }
        for (ScenarioConfig.TunnelConfig t : c.road.tunnels) {
            double len = t.to - t.from;
            if (t.from < 0 || t.to > mainLength || len <= 0) {
                errors.add("tunnel [" + t.from + ", " + t.to + "] must lie within the main road (0.." + mainLength + ")");
            }
        }
        for (ScenarioConfig.BranchConfig b : c.road.branches) {
            if (b.at <= 0 || b.at >= mainLength) {
                errors.add("branch at " + b.at + " must lie strictly inside the main road");
            }
            if (b.ramp.isEmpty()) {
                errors.add("branch at " + b.at + " needs ramp segments");
            }
            for (ScenarioConfig.SegmentConfig s : b.ramp) {
                checkSegment(s, "branch ramp", errors);
            }
        }
        if (c.cars.isEmpty()) {
            errors.add("at least one car is required");
        }
        if (!List.of("GENERIC", "IOS", "LOW_END").contains(c.device)) {
            errors.add("device must be GENERIC, IOS or LOW_END");
        }
        for (ScenarioConfig.CarConfig car : c.cars) {
            for (ScenarioConfig.EventConfig e : car.events) {
                if (!List.of("slowTo", "stop", "stopAndGo", "hardBrake", "mountShift").contains(e.type)) {
                    errors.add("car " + car.id + ": unknown event type '" + e.type + "'");
                }
            }
        }
        if (!errors.isEmpty()) {
            throw new IllegalArgumentException("Invalid scenario '" + c.name + "':\n  " + String.join("\n  ", errors));
        }
        return c;
    }

    private static void checkSegment(ScenarioConfig.SegmentConfig s, String where, List<String> errors) {
        if (!List.of("straight", "arc", "clothoid").contains(s.type)) {
            errors.add(where + ": unknown segment type '" + s.type + "'");
        }
        if (s.length <= 0) {
            errors.add(where + ": segment length must be positive");
        }
        if ("arc".equals(s.type) && s.curvature == null && s.radius == null) {
            errors.add(where + ": arc needs curvature or radius");
        }
    }
}
