package tunnelsim.sensor;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;

/** All sensor events of one phone, sorted by time. */
public final class SensorLog {
    private final List<SensorEvent> events;

    public SensorLog(Collection<? extends List<? extends SensorEvent>> streams) {
        List<SensorEvent> all = new ArrayList<>();
        streams.forEach(all::addAll);
        all.sort(Comparator.comparingDouble(SensorEvent::t)); // stable: keeps stream order on ties
        this.events = List.copyOf(all);
    }

    public List<SensorEvent> events() {
        return events;
    }

    public <T extends SensorEvent> List<T> of(Class<T> type) {
        List<T> out = new ArrayList<>();
        for (SensorEvent e : events) {
            if (type.isInstance(e)) {
                out.add(type.cast(e));
            }
        }
        return out;
    }
}
