package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

public class TestGoalLock extends TorqueSequence implements Subsystems {
    public TestGoalLock() {
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
        addBlock(new TorqueFollowPath("test_goal_lock", drivebase));
    }
    
}
