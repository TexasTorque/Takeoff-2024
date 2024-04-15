package org.texastorque.auto.routines;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.texastorque.Subsystems;
import org.texastorque.auto.NoteSequence;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Intake.State;

import com.pathplanner.lib.path.PathPlannerPath;
import org.texastorque.Field;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * This sequence runs a path from one note to another.
 */
public class CollectAndShootNote extends TorqueSequence implements Subsystems {

    /**
     * Creats a torque follow path command using a path supplier and provides our
     * drivebase.
     */
    public static TorqueFollowPath followPath(final Supplier<PathPlannerPath> path, final BooleanSupplier endsEarly) {
        return new TorqueFollowPath(path, drivebase, endsEarly);
    }

    private boolean deployIntakeRightAway = true; // init val doesnt matter
    private boolean isFarSide = false;
    private boolean dangerous = false;
    private boolean endedEarly = false;
    private boolean lastEndedEarly = false;
    private int timesTried = 0;

    private boolean endsEarly() {
        // if (!perception.isUsingDetectionLock())
        // return false;
        // if (deployIntakeRightAway)
        // return false;
        // if (timesTried > 3)
        // return false;
        // endedEarly = perception.hasNoCloseNotes();
        // timesTried++;
        // return endedEarly;
        endedEarly = false;
        return false;
    }

    public CollectAndShootNote(final NoteSequence noteSequence) {

        addBlock(new TorqueRun(() -> lastEndedEarly = noteSequence.peekNext().fragmented));

        // Peek the next note pair and collect some data on it
        addBlock(new TorqueRun(() -> deployIntakeRightAway = !noteSequence.peekNext().end.isMidline()));
        addBlock(new TorqueRun(() -> isFarSide = noteSequence.peekNext().end.isFarSide()));
        // // Set paths that are "dangerous"
        // addBlock(new TorqueRun(() -> {
        // int start = noteSequence.peekNext().start().getID();
        // int end = noteSequence.peekNext().end().getID();
        // dangerous = (start == 40 && end == 30);
        // }));

        log("Can Deploy Intake", () -> deployIntakeRightAway);

        log("Auto State", () -> "BEGIN PATH");

        // WARNING: THIS IS THE POP!!! -- any subsequent peeks will be for the next
        // note!
        addBlock(followPath(() -> noteSequence.popNext().getPath(), this::endsEarly),
                new DeployIntakeWhen(
                        () -> {
                            return field.isXPast(perception.getPose(), 5.25) || deployIntakeRightAway || lastEndedEarly;
                        },
                        () -> endedEarly).command(),
                new NoteAlign(
                        () -> !deployIntakeRightAway,
                        () -> {
                            return deployIntakeRightAway || field.isXPast(perception.getPose(), 6) || lastEndedEarly;
                        }).command());

        log("isFarSide", () -> isFarSide);

        addBlock(new TorqueRun(() -> field.useFarSideSpeaker(isFarSide)));
        addBlock(new TorqueRunSequence(new Shoot(Shooter.State.FUTURE_SMART_ALIGN)));

        addBlock(new TorqueRun(() -> {
            if (endedEarly) {
                noteSequence.makeNextFragmented();
            }
        }));
    }
}