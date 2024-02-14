package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;

public class RunPath extends TorqueSequence implements Subsystems {
    public RunPath() {
        addBlock(new TorqueRun(() -> perception.resetPose()));
        addBlock(new TorqueFollowPath("line", drivebase));
    }
    
}
