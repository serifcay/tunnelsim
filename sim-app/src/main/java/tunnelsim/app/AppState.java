package tunnelsim.app;

import tunnelsim.engine.CarResult;
import tunnelsim.engine.RunResult;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/** What all views draw: the current run, the playhead, the selected car and visible estimators. */
final class AppState {
    RunResult result;
    double time;
    int car;
    final Set<String> visible = new LinkedHashSet<>();
    boolean showParticles = true;
    boolean showDisplayMarker = false;
    private final List<Runnable> listeners = new ArrayList<>();

    CarResult car() {
        return result == null || result.cars().isEmpty() ? null : result.cars().get(Math.min(car, result.cars().size() - 1));
    }

    void onChange(Runnable r) {
        listeners.add(r);
    }

    void changed() {
        listeners.forEach(Runnable::run);
    }
}
