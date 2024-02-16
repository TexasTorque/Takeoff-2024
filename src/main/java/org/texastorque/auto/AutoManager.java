package org.texastorque.auto;

import org.texastorque.auto.sequences.BaseAuto;
import org.texastorque.auto.sequences.RunPath;
import org.texastorque.auto.sequences.TestGoalLock;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        addSequence("2", new BaseAuto(2));

        addSequence("2 to 1", new BaseAuto(2, 1));
        addSequence("2 to 1 to 10", new BaseAuto(2, 1, 10));
        addSequence("2 to 1 to 10 to 20", new BaseAuto(2, 1, 10, 20));

        addSequence("2 to 3", new BaseAuto(2, 3));
        addSequence("2 to 3 to 50", new BaseAuto(2, 3, 50));
        addSequence("2 to 3 to 50 to 40", new BaseAuto(2, 3, 50, 40));

        addSequence(new RunPath());

        addSequence(new TestGoalLock());
    }

    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}