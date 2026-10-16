package tunnelsim.util;

import java.util.SplittableRandom;

/**
 * A seeded random stream. Every simulation component gets its own stream derived from the
 * master seed and a stable name, so changing one component's parameters never changes the
 * noise drawn by another.
 */
public final class Rng {
    private final SplittableRandom random;

    public Rng(long seed) {
        this.random = new SplittableRandom(seed);
    }

    /** Derives an independent stream from a master seed and a component name. */
    public static Rng derive(long masterSeed, String name) {
        long h = masterSeed ^ 0x9E3779B97F4A7C15L;
        for (int i = 0; i < name.length(); i++) {
            h = mix(h + name.charAt(i));
        }
        return new Rng(mix(h));
    }

    private static long mix(long z) {
        z = (z ^ (z >>> 30)) * 0xBF58476D1CE4E5B9L;
        z = (z ^ (z >>> 27)) * 0x94D049BB133111EBL;
        return z ^ (z >>> 31);
    }

    public double gaussian() {
        return random.nextGaussian();
    }

    public double gaussian(double sigma) {
        return sigma * random.nextGaussian();
    }

    public double uniform() {
        return random.nextDouble();
    }

    public double uniform(double min, double max) {
        return min + (max - min) * random.nextDouble();
    }

    public int nextInt(int bound) {
        return random.nextInt(bound);
    }

    public boolean chance(double p) {
        return random.nextDouble() < p;
    }

    /** Uniform in [lo, hi] given as a two-element range; a single value is returned as is. */
    public double inRange(double[] range) {
        if (range == null || range.length == 0) {
            return 0;
        }
        return range.length == 1 ? range[0] : uniform(range[0], range[1]);
    }
}
