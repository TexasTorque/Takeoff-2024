/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import java.util.Optional;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Shoot;
import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.auto.sequences.Line;
import org.texastorque.auto.NoteSequence.Adapative;
import org.texastorque.auto.NoteSequence.Location;
import org.texastorque.auto.NoteSequence.LocationPair;
import org.texastorque.auto.NoteSequence.NotePoint;
import org.texastorque.auto.NoteSequence.StartPoint;
import org.texastorque.subsystems.Shooter;
import org.texastorque.torquelib.auto.*;
import com.pathplanner.lib.path.PathPlannerPath;

/** Manage the auto loader and selections */
public final class AutoManager extends TorqueAutoManager implements Subsystems {
    private static volatile AutoManager instance;

    /**
     * Preload all the paths so that we dont make expensive reasource loader calls
     * when
     * the auto is suposed to be going fast!
     */
    @Override
    public final void loadPaths() {

        // Generate the following code using
        // > python3 listpaths.py
        pathLoader.preloadPath("go_3_to_2");
        pathLoader.preloadPath("go_10_to_20");
        pathLoader.preloadPath("go_NONE_to_NONE");
        pathLoader.preloadPath("go_SRC_to_50");
        pathLoader.preloadPath("go_AMP_to_1");
        pathLoader.preloadPath("go_2_to_3");
        pathLoader.preloadPath("go_1_to_10");
        pathLoader.preloadPath("go_1_to_2");
        pathLoader.preloadPath("go_CTR_to_3");
        pathLoader.preloadPath("go_CTR_to_2");
        pathLoader.preloadPath("go_50_to_40");
        pathLoader.preloadPath("go_40_to_3");
        pathLoader.preloadPath("go_2_to_1");
        pathLoader.preloadPath("line");
        pathLoader.preloadPath("go_1_to_20");
        pathLoader.preloadPath("go_3_to_50");
    }

    // /**
    // * Get a preloaded path... the current path strategy is EXPLICITLY UNSAFE...
    // * ...if a path is called that is not loaded above then the program WILL FAIL!
    // *
    // * Helpful for debugging, but not for production!!!!
    // */
    // public final PathPlannerPath getPath(final String pathName) {
    // System.out.println("Loading path " + pathName);
    // return pathLoader.getPathUnsafe(pathName);
    // }

    /**
     * Get a preloaded path. If the paths is not loaded then we return
     * the none path.
     */
    public final PathPlannerPath getPath(final String pathName) {
        final Optional<PathPlannerPath> pathOpt = pathLoader.getPathSafe(pathName);
        if (pathOpt.isPresent()) {
            return pathOpt.get();
        }
        System.out.println("Failed to load path " + pathName);
        return pathLoader.getPathUnsafe("go_NONE_to_NONE");
    }

    /** Load all the permutations of auto sequences we want to run */
    @Override
    public final void loadSequences() {

        // Just shoot auto
        addSequence(new Shoot(Shooter.State.LAYUP));
        addSequence(new Line());

        // Amp side only
        addBaseAuto(StartPoint.AMP, NotePoint.N_1, NotePoint.N_10, NotePoint.N_20);
        addBaseAuto(StartPoint.AMP, NotePoint.N_1, NotePoint.N_20);

        // Clear center area
        addBaseAuto(StartPoint.CTR, NotePoint.N_2, NotePoint.N_1);
        addBaseAuto(StartPoint.CTR, NotePoint.N_3, NotePoint.N_2);
        addBaseAuto(StartPoint.CTR, NotePoint.N_3, NotePoint.N_2, NotePoint.N_1);
        addBaseAuto(StartPoint.CTR, NotePoint.N_3, NotePoint.N_2, NotePoint.N_1, NotePoint.N_10);

        // Far side capable
        addBaseAuto(StartPoint.CTR, NotePoint.N_2, NotePoint.N_3, NotePoint.N_50);
        addBaseAuto(StartPoint.SRC, NotePoint.N_3, NotePoint.N_50);
        addBaseAuto(StartPoint.SRC, NotePoint.N_50, NotePoint.N_40);
        addBaseAuto(StartPoint.SRC, NotePoint.N_50, NotePoint.N_40, NotePoint.N_3);
    }

    /** Create a base auto and come up with a name for it */
    private void addBaseAuto(final Location... notes) {
        final NoteSequence ns = new NoteSequence(notes);
        addSequence(ns.toString(), new BaseAuto(ns));
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
