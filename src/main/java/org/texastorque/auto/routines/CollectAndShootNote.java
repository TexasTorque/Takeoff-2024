package org.texastorque.auto.routines;

import java.util.function.Supplier;

import org.texastorque.Subsystems;
import org.texastorque.auto.NoteSequence;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.subsystems.*;
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

    private boolean isFarSide = false;

    public CollectAndShootNote(final NoteSequence noteSequence) {
        // Peek the next note pair and collect some data on it
        addBlock(new TorqueRun(() -> deployIntakeRightAway = !noteSequence.peekNext().end().isFarSide()));
        addBlock(new TorqueRun(() -> isFarSide = noteSequence.peekNext().end().isFarSide()));

        log("Can Deploy Intake", () -> deployIntakeRightAway);

        log("Auto State", () -> "BEGIN PATH");

        // WARNING: THIS IS THE POP!!! -- any subsequent peeks will be for the next note!
        addBlock(followPath(() -> noteSequence.popNext().getPath()),
                new DeployIntakeWhen(() -> field.isXPast(perception.getPose(), 4.5) || deployIntakeRightAway)
                        .command());

        addBlock(new TorqueRunSequence(new Shoot(deployIntakeRightAway ? Shooter.State.FUTURE_SMART : Shooter.State.FUTURE_SMART)));
    }
}