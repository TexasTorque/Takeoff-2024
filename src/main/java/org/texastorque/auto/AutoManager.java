/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.auto.sequences.Dash;
import org.texastorque.torquelib.auto.*;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

/** Manage the auto loader and selections */
public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    /**
     * Preload all the paths so that we dont make expensive reasource loader calls when
     * the auto is suposed to be going fast!
     */
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

        pathLoader.preloadPath("go_3_to_50");

        pathLoader.preloadPath("go_0_to_50");
        pathLoader.preloadPath("go_50_to_40");
        pathLoader.preloadPath("go_40_to_30");

        pathLoader.preloadPath("go_1_to_10");
        pathLoader.preloadPath("go_1_to_15");
        pathLoader.preloadPath("go_10_to_20");
        pathLoader.preloadPath("line");
    }

    /** 
     * Get a preloaded path... the current path strategy is EXPLICITLY UNSAFE...
     * ...if a path is called that is not loaded above then the program WILL FAIL!
     * 
     * Helpful for debugging, but not for production!!!!
     */
    public final PathPlannerPath getPath(final String pathName) {
        return pathLoader.getPathUnsafe(pathName);
    }

    /** Load all the permutations of auto sequences we want to run */
    @Override
    public final void loadSequences() {
        addSequence("0", new BaseAuto(new Pose2d(1.42, 6.35, perception.getHeading())));

        addSequence("1 to 2", new BaseAuto(new Pose2d(1.42, 6.35, perception.getHeading()), 1, 2));
        addSequence("1 to 2 to 3", new BaseAuto(new Pose2d(1.42, 6.35, perception.getHeading()), 1, 2, 3));

        addSequence("1 to 10", new BaseAuto(new Pose2d(0.77, 6.58, perception.getHeading()), 1, 10));

        addSequence("1 to 10 to 20", new BaseAuto(new Pose2d(0.77, 6.58, perception.getHeading()), 1, 10, 20));

        addSequence("2 to 1", new BaseAuto(new Pose2d(1.33, 5.55, perception.getHeading()), 2, 1));
        addSequence("2 to 3", new BaseAuto(new Pose2d(1.33, 5.55, perception.getHeading()), 2, 3));
        addSequence("2 to 3 to 50", new BaseAuto(new Pose2d(1.33, 5.55, perception.getHeading()), 2, 3, 50));
        // addSequence("2 to 1 to 10 to 20", new BaseAuto(2, 1, 10, 20));

        addSequence("3 to 2", new BaseAuto(new Pose2d(1.28, 4.7, perception.getHeading()), 3, 2));
        addSequence("3 to 2 to 1", new BaseAuto(new Pose2d(1.28, 4.7, perception.getHeading()), 3, 2, 1));
        addSequence("3 to 2 to 1 to 10", new BaseAuto(new Pose2d(1.28, 4.7, perception.getHeading()), 3, 2, 1, 10));
        addSequence("3 to 50", new BaseAuto(new Pose2d(1.25, 5.37, perception.getHeading()), 3, 50));
        // addSequence("3 to 2 to 1 to 15", new BaseAuto(new Pose2d(1.28, 4.7, perception.getHeading()), 3, 2, 1, 15));

        // addSequence("10 to 20 to 30", new BaseAuto(10, 20, 30));

        // addSequence("50 to 40 to 30", new BaseAuto(50, 40, 30));

        // addSequence("DASH to 10 to 20", new Dash(10, 20));

        addSequence("10 to 20", new BaseAuto(new Pose2d(), 10, 20));
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
