package org.texastorque.auto;

import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.torquelib.auto.*;

import com.pathplanner.lib.path.PathPlannerPath;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void loadPaths() {
        pathLoader.preloadPath("go_0_to_2");
        pathLoader.preloadPath("go_0_to_10");
        pathLoader.preloadPath("go_10_to_20");
        pathLoader.preloadPath("go_20_to_30");

        pathLoader.preloadPath("go_0_to_1");
        pathLoader.preloadPath("go_1_to_2");
        pathLoader.preloadPath("go_2_to_3");

        pathLoader.preloadPath("go_0_to_3");
        pathLoader.preloadPath("go_3_to_2");
        pathLoader.preloadPath("go_2_to_1");

        pathLoader.preloadPath("go_0_to_50");
        pathLoader.preloadPath("go_50_to_40");
        pathLoader.preloadPath("go_40_to_30");

        pathLoader.preloadPath("go_1_to_10");
        pathLoader.preloadPath("go_10_to_20");
    }

    public final PathPlannerPath getPath(final String pathName) {
        return pathLoader.getPathUnsafe(pathName);
    }

    @Override
    public final void loadSequences() {
        addSequence("1 to 2", new BaseAuto(1, 2));
        addSequence("1 to 2 to 3", new BaseAuto(1, 2, 3));

        addSequence("2 to 1", new BaseAuto(2, 1));
        addSequence("2 to 3", new BaseAuto(2, 3));
        addSequence("2 to 1 to 10 to 20", new BaseAuto(2, 1, 10, 20));

        addSequence("3 to 2", new BaseAuto(3, 2));
        addSequence("3 to 2 to 1", new BaseAuto(3, 2, 1));
        addSequence("3 to 2 to 1 to 10", new BaseAuto(3, 2, 1, 10));

        addSequence("10 to 20 to 30", new BaseAuto(10, 20, 30));

        addSequence("50 to 40 to 30", new BaseAuto(50, 40, 30));
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
