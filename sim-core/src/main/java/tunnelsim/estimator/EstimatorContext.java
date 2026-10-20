package tunnelsim.estimator;

import tunnelsim.map.RoadNetwork;
import tunnelsim.map.Route;
import tunnelsim.scenario.EstimatorConfig;
import tunnelsim.scenario.ScenarioConfig;
import tunnelsim.util.Rng;

/** Everything an estimator may know: the map, the route, its configuration and a random stream. */
public record EstimatorContext(RoadNetwork net, Route route, ScenarioConfig scenario, EstimatorConfig config,
                               boolean optionE, Rng rng) {

    public EstimatorConfig.AidingConfig aiding() {
        return config.aiding;
    }

    /** BLE is used only by the option-E estimator, when the scenario has beacons and the aiding is on. */
    public boolean useBle() {
        return optionE && scenario.options.ble && config.aiding.ble;
    }

    /** OBD speed is used only by the option-E estimator, when the scenario has it and the aiding is on. */
    public boolean useObd() {
        return optionE && scenario.options.obd && config.aiding.obd;
    }
}
