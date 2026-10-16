package tunnelsim.batch;

import tunnelsim.engine.RunResult;
import tunnelsim.engine.SimulationRun;
import tunnelsim.metrics.MetricsTable;
import tunnelsim.metrics.MonteCarlo;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.scenario.ScenarioLoader;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Command-line runner.
 * <pre>
 *   single  [scenario.yaml ...]          one run per scenario, metrics table (default: all scenarios)
 *   mc      [-n RUNS] [scenario.yaml ...] Monte Carlo summary per scenario
 * </pre>
 */
public class BatchMain {
    public static void main(String[] args) {
        String mode = args.length > 0 && !args[0].endsWith(".yaml") ? args[0] : "single";
        List<String> rest = new ArrayList<>(List.of(args));
        if (!rest.isEmpty() && rest.get(0).equals(mode)) {
            rest.remove(0);
        }
        int runs = 50;
        if (rest.size() >= 2 && rest.get(0).equals("-n")) {
            runs = Integer.parseInt(rest.get(1));
            rest = rest.subList(2, rest.size());
        }
        List<Path> files = rest.isEmpty()
                ? ScenarioLoader.list(Path.of("scenarios"))
                : rest.stream().map(Path::of).toList();

        switch (mode) {
            case "single" -> {
                for (Path file : files) {
                    RunResult run = SimulationRun.run(ScenarioLoader.load(file));
                    System.out.println(MetricsTable.format(run));
                }
            }
            case "mc" -> {
                for (Path file : files) {
                    ScenarioConfig cfg = ScenarioLoader.load(file);
                    long t0 = System.nanoTime();
                    MonteCarlo.Result r = MonteCarlo.run(cfg, runs);
                    System.out.print(MonteCarlo.format(r));
                    System.out.printf("  (%.1f s)%n%n", (System.nanoTime() - t0) / 1e9);
                }
            }
            default -> {
                System.err.println("Unknown mode '" + mode + "'. Use: single | mc [-n RUNS]");
                System.exit(2);
            }
        }
    }
}
