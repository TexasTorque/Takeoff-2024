package org.texastorque.auto.sequences;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Shooter.GateState;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class BaseAuto extends TorqueSequence implements Subsystems {

    /**
     * Creats a torque follow path command using a path supplier and provides our
     * drivebase.
     */
    public static TorqueFollowPath followPath(final Supplier<PathPlannerPath> path) {
        return new TorqueFollowPath(path, drivebase);
    }

    /**
     * A NoteSequence is a list of notes *indexes* (not actual Note objects) that is
     * encapsulated
     * so that we can calculate paths.
     * 
     * Note indexes work as so. The first three notes that are placed inside the
     * alliance wing
     * are indexed from top down 1, 2, and 3. The notes on the center line are
     * indexed from top
     * down 10, 20, 30, 40, 50. Close notes are indexed < 10, center line notes are
     * indexed >= 10.
     */
    private class NoteSequence {
        private final List<Integer> notes = new ArrayList<Integer>();
        int lastNote = 0, nextNote = 0;
        String pathName = "";

        /**
         * Creates a note sequence from a variatic list of arguments which provide
         * indexes.
         */
        public NoteSequence(final int... notes) {
            for (int note : notes) {
                this.notes.add(note);
            }
        }

        /**
         * Calculates the name of and loads the path that will take the robot from the
         * current
         * note we are at to the next note in the sequence.
         * 
         * @return Some PathPlannerPath object that we should follow.
         */
        private PathPlannerPath getNextPath() {
            lastNote = nextNote;
            nextNote = notes.remove(0);
            pathName = "go_" + lastNote + "_to_" + nextNote;
            return PathPlannerPath.fromPathFile(pathName);
        }

        private Pose2d getPathEndPose() {
            return PathPlannerPath.fromPathFile(pathName).getTrajectory(new ChassisSpeeds(), new Rotation2d())
                    .getEndState().getTargetHolonomicPose();
        }

        /**
         * Take a peek at the index of the next note in the sequence, but do not remove
         * it.
         * 
         * @return The next note's index.
         */
        public int peekNext() {
            return notes.get(0);
        }

        /**
         * Is the next note index a center line note (the index is >= 10)?
         */
        public boolean isNextOnCenterLine() {
            return peekNext() >= 10;
        }

        /**
         * Do we have another note in our sequence?
         */
        public boolean hasNext() {
            return notes.size() > 0;
        }
    }

    /**
     * This should shoot the gamepeice using smartshot, therefor aligning drivebase,
     * and will wait until the shooter is ready + a small delay for the peice to
     * leave.
     */
    public class Shoot extends TorqueSequence {
        private final double waitTime = .2;

        public Shoot() {
            log("Auto State", () -> "SHOOTING");

            addBlock(shooter.yieldState(Shooter.State.FUTURE_SMART));
            if (RobotBase.isReal()) {
                addBlock(new TorqueWaitUntil(() -> !shooter.hasNote()));
            } else {
                addBlock(new TorqueWaitTime(1));
            }
            // addBlock(new TorqueWaitTime(waitTime));
            addBlock(shooter.yieldState(Shooter.State.AUTO_OFF));
            addBlock(shooter.yieldGateState(Shooter.GateState.OFF));
            log("Auto State", () -> "SHOT");
        }
    }

    /**
     * A sequence for deploying the intake after some boolean condition,
     * provided with a BooleanSupplier yields true.
     */
    public class DeployIntakeWhen extends TorqueSequence {
        private Timer timer = new Timer();

        public DeployIntakeWhen(final BooleanSupplier when) {
            addBlock(new TorqueWaitUntil(when));
            addBlock(new TorqueRun(() -> timer.restart()));

            log("Auto State", () -> "INTAKING");

            addBlock(intake.yieldState(Intake.State.SMART_INTAKE), shooter.yieldState(Shooter.State.OFF));

            if (RobotBase.isReal()) {
                addBlock(new TorqueWaitUntil(intake::isAtState));
            }

            addBlock(shooter.yieldState(Shooter.State.INTAKE));
            addBlock(shooter.yieldGateState(Shooter.GateState.IN));

            if (RobotBase.isReal()) {
                addBlock(new TorqueWaitUntil(() -> shooter.hasNote() || timer.get() > 3));
            } else {
                addBlock(new TorqueWaitTime(2));
            }

            log("Auto State", () -> "WARMING UP");

            addBlock(shooter.yieldGateState(GateState.OFF));
            addBlock(new TorqueRun(() -> perception.setFutureShootingPose(noteSequence.getPathEndPose())));
            addBlock(shooter.yieldState(Shooter.State.FUTURE_SMART));

            addBlock(intake.yieldState(Intake.State.PRIME));
        }
    }

    /**
     * This sequence runs a path from one note to another.
     */
    public class CollectAndShootNote extends TorqueSequence {
        private boolean deployIntakeRightAway = true; // init val doesnt matter

        public CollectAndShootNote(final NoteSequence noteSequence) {
            addBlock(new TorqueRun(() -> deployIntakeRightAway = !noteSequence.isNextOnCenterLine()));

            log("Can Deploy Intake", () -> deployIntakeRightAway);

            log("Auto State", () -> "BEGIN PATH");

            addBlock(followPath(() -> noteSequence.getNextPath()),
                    new DeployIntakeWhen(() -> perception.getPose().getX() > 6 || deployIntakeRightAway).command());

            // addBlock(new TorqueWaitTime(() -> isNextOnCenterLine ? .25 : 0));

            addBlock(new TorqueRunSequence(new Shoot()));
        }
    }

    private final NoteSequence noteSequence;

    public BaseAuto(final int... notes) {
        noteSequence = new NoteSequence(notes);

        // addBlock(new TorqueRun(() -> perception.setPose(new Pose2d(1.33, 5.55, Field.ROT_FWD))));
        // addBlock(new TorqueRun(() -> perception.resetGyro()));

        addBlock(new TorqueRun(() -> perception.setFutureShootingPose(perception.getPose())));
        addBlock(new TorqueRunSequence(new Shoot()));

        addBlock(new TorqueWhile(noteSequence::hasNext, new CollectAndShootNote(noteSequence)));
    }

}