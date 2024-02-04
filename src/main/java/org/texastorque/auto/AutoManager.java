package org.texastorque.auto;

import org.texastorque.auto.sequences.DynamicAuto;
import org.texastorque.auto.sequences.PathTest;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        // setConstAuto(new DynamicAuto()); // for now this is the only we will run

        addSequence(new DynamicAuto());
        addSequence(new PathTest());
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}