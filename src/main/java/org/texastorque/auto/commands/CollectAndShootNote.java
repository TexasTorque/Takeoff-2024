package org.texastorque.auto.commands;

import java.util.function.Supplier;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This sequence runs a path from one note to another.
 */
public class CollectAndShootNote extends TorqueSequence implements Subsystems {

    /**
     * Creats a torque follow path command using a path supplier and provides our
     * drivebase.
     */
    public static TorqueFollowPath followPath(final Supplier<PathPlannerPath> path) {
        return new TorqueFollowPath(path, drivebase);
    }

    private boolean deployIntakeRightAway = true; // init val doesnt matter

    public CollectAndShootNote(final NoteSequence noteSequence) {
        addBlock(new TorqueRun(() -> deployIntakeRightAway = !noteSequence.isNextOnCenterLine()));

        log("Can Deploy Intake", () -> deployIntakeRightAway);

        log("Auto State", () -> "BEGIN PATH");

        addBlock(followPath(() -> noteSequence.getNextPath()),
                new DeployIntakeWhen(() -> field.isXPast(perception.getPose(), 5.5) || deployIntakeRightAway)
                        .command());

        addBlock(new TorqueRunSequence(new Shoot()));
    }
}