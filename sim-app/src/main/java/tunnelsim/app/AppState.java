package tunnelsim.app;

import tunnelsim.engine.CarResult;
import tunnelsim.engine.RunResult;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/** What all views draw: the current run, the playhead, the selected car and visible estimators. */
final class AppState {
    // Current run, playhead time, index of the selected car, and the estimators that are shown.
    RunResult result;
    double time;
    int car;
    final Set<String> visible = new LinkedHashSet<>();
    boolean showParticles = true;
    boolean showDisplayMarker = false;
    private final List<Runnable> listeners = new ArrayList<>();

    /** The selected car's result (null before the first run). */
    CarResult car() {
        return result == null || result.cars().isEmpty() ? null : result.cars().get(Math.min(car, result.cars().size() - 1));
    }

    void onChange(Runnable r) {
        listeners.add(r);
    }

    /** Tells every view that the state changed (e.g. the playhead moved), so they redraw. */
    void changed() {
        listeners.forEach(Runnable::run);
    }
}
