package tunnelsim.engine;

import tunnelsim.estimator.Estimate;
import tunnelsim.nav.CalloutTracker.Callout;

import java.util.ArrayList;
import java.util.List;

/** One estimator's recorded output for one car, sampled at 10 Hz. */
public final class Track {
    /**
     * @param error          signed along-road error of the estimate [m] (+ = ahead of the car)
     * @param wrongBranch    estimate on a different branch than the car
     * @param pTruthPath     probability the estimator gives to the car's true path
     * @param pTruthBranch   probability of the branch the car took at the last fork it passed
     * @param displayOffset  path offset of the displayed marker on {@code displayLeaf}'s path
     * @param displayError   signed along-road error of the displayed marker [m]
     */
    public record Sample(double t, Estimate estimate, double error, boolean wrongBranch, double pTruthPath,
                         double pTruthBranch,
                         int displayLeaf, double displayOffset, double displayError, boolean displayWrong) {
    }

    public final String estimatorId;
    public final List<Sample> samples = new ArrayList<>();
    public final List<Callout> callouts = new ArrayList<>();
    public final List<RunEvent> events = new ArrayList<>();
    public long nanos;
    public int reinitialisations;

    public Track(String estimatorId) {
        this.estimatorId = estimatorId;
    }

    /** Sample at or just before time t. */
    public Sample at(double t) {
        if (samples.isEmpty()) {
            return null;
        }
        // Samples are recorded every RECORD_DT seconds from t = 0, so the index follows from the time.
        int i = (int) Math.floor(t / RunConstants.RECORD_DT + 1e-9);
        return samples.get(Math.max(0, Math.min(samples.size() - 1, i)));
    }
}
