package org.texastorque.auto.sequences;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Perception.Note;
import org.texastorque.toast.lib.Util;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueUtil;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;

public class BaseAuto extends TorqueSequence implements Subsystems {

    /**
     * Creats a torque follow path command using a path supplier and provides our drivebase. 
     */
    public static TorqueFollowPath followPath(final Supplier<PathPlannerPath> path) {
        return new TorqueFollowPath(path, drivebase);
    }

    /**
     * A NoteSequence is a list of notes *indexes* (not actual Note objects) that is encapsulated 
     * so that we can calculate paths.
     * 
     * Note indexes work as so. The first three notes that are placed inside the alliance wing
     * are indexed from top down 1, 2, and 3. The notes on the center line are indexed from top
     * down 10, 20, 30, 40, 50. Close notes are indexed < 10, center line notes are indexed >= 10.
     */
    private class NoteSequence {
        private final List<Integer> notes = new ArrayList<Integer>();
        int lastNote = 0, nextNote = 0;

        /**
         * Creates a note sequence from a variatic list of arguments which provide indexes.
         */
        public NoteSequence(final int... notes) {
            for (int note : notes) {
                this.notes.add(note);
            }
        }
     
        /**
         * Calculates the name of and loads the path that will take the robot from the current 
         * note we are at to the next note in the sequence.
         * 
         * @return Some PathPlannerPath object that we should follow.
         */
        private PathPlannerPath getNextPath() {
            lastNote = nextNote;
            nextNote = notes.remove(0);
            final String pathName = "go_" + lastNote + "_to_" + nextNote;
            return PathPlannerPath.fromPathFile(pathName);
        }

        /**
         * Take a peek at the index of the next note in the sequence, but do not remove it.
         * 
         * @return The next note's index.
         */
        public int peekNext() { return notes.get(0); }

        /**
         * Is the next note index a center line note (the index is >= 10)?
         */
        public boolean isPeekCenterLine() { return peekNext() >= 10; }

        /**
         * Do we have another note in our sequence?
         */
        public boolean hasNext() { return notes.size() > 0; }
    }

    /**
     * This should shoot the gamepeice using smartshot, therefor aligning drivebase,
     * and will wait until the shooter is ready + a small delay for the peice to leave.
     */
    public class Shoot extends TorqueSequence {
        private final double waitTime = .5;

        public Shoot() {
            // addBlock(shooter.yieldState(Shooter.State.SMART));
            // addBlock(new TorqueWaitUntil(shooter::isReadyToShoot));
            // addBlock(new TorqueWaitTime(waitTime));
            // addBlock(shooter.yieldState(Shooter.State.OFF));
            // addBlock(intake.yieldState(Intake.State.OFF));
            addBlock(new TorqueWaitTime(1));
        }
    }

    /**
     * A sequence for deploying the intake after some boolean condition, 
     * provided with a BooleanSupplier yields true.
     */
    public class DeployIntakeWhen extends TorqueSequence {
        public DeployIntakeWhen(final BooleanSupplier when) {
            addBlock(new TorqueWaitUntil(when));
            addBlock(intake.yieldState(Intake.State.SMART_INTAKE));
        }
    }

    /**
     * This sequence runs a path from one note to another. 
     */
    public class CollectAndShootNote extends TorqueSequence {
        public CollectAndShootNote(final NoteSequence noteSequence) {

            // If we are going to the center line we wait until our X coord is > 5.5
            // to deploy the intake. Otherwise we just do it right now.
            final BooleanSupplier deployIntakeWhen = noteSequence.isPeekCenterLine()
                ? () -> perception.getPose().getX() > 5.5 : () -> true;

            addBlock(followPath(() -> noteSequence.getNextPath()), 
                new DeployIntakeWhen(deployIntakeWhen).command());

            addBlock(new TorqueWaitUntil(shooter::hasNote));
            addBlock(new TorqueRunSequence(new Shoot()));
        }
    }

    private final NoteSequence noteSequence;

    public BaseAuto(final int... notes) {
        noteSequence = new NoteSequence(notes);

        // This is debug only
        addBlock(new TorqueRun(() -> perception.setPose(new Pose2d(1.1, 5.75, Field.ROT_FWD))));

        // addBlock(shooter.yieldState(Shooter.State.WARMUP));
        addBlock(new TorqueRunSequence(new Shoot()));

        addBlock(new TorqueWhile(noteSequence::hasNext, new CollectAndShootNote(noteSequence)));
    }

}