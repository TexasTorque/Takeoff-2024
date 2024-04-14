/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;

/** For debugging */
public class Line extends TorqueSequence implements Subsystems {
    public Line() {
        addBlock(new TorqueRun(() -> perception.useDetectionLock(true)));
        addBlock(new TorqueRun(() -> perception.resetPose()));
        addBlock(new TorqueFollowPath("line", drivebase));
        addBlock(new TorqueWaitTime(0.5));
        addBlock(new TorqueRun(() -> perception.useDetectionLock(false)));
    }

}
