package org.texastorque.auto;

import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        // addSequence("1", new BaseAuto(1));
        addSequence("2", new BaseAuto(2));
        // addSequence("3", new BaseAuto(3));
        addSequence("2 to 1", new BaseAuto(2, 1));
        // addSequence("2 to 3", new BaseAuto(2, 3));
        // addSequence("1 to 2 to 3", new BaseAuto(1, 2, 3));
        // addSequence("3 to 2 to 1", new BaseAuto(3, 2, 1));
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