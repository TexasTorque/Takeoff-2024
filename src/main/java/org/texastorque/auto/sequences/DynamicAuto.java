package org.texastorque.auto.sequences;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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

public class DynamicAuto extends TorqueSequence implements Subsystems {

    public static TorqueFollowPath followPath(final Supplier<PathPlannerPath> path) {
        return new TorqueFollowPath(path, drivebase, 3);
    }
  
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

    public class GetAndScoreCloseNote extends TorqueSequence {
        // TODO: this may have to be supplier, idk
        public GetAndScoreCloseNote(final AutoConfig config) {
            addBlock(intake.yieldState(Intake.State.SMART_INTAKE));
            addBlock(followPath(() -> perception.generateNextOmar(config.getNextNote())));
            addBlock(new TorqueWaitUntil(shooter::hasGateSpiked));
            addBlock(new TorqueRunSequence(new Shoot()));
        }
    }

    public class HandleCenterLineNotes extends TorqueSequence {

        private Pose2d homingPose = new Pose2d(); // this can be safely set to nothing because...
        private Pose2d shootingPose = new Pose2d(); // (same thing here)

        private Note bestNote = Note.EMPTY;

        private boolean stopIntaking() {
            return shooter.hasGateSpiked() || perception.getPose().getX() >= Field.LENGTH / 2 + 0;
        }

        public HandleCenterLineNotes() {
            // ... this line will pass
            addBlock(new TorqueRun(() -> homingPose = perception.isAboveSpeakerOnY() ? Field.HOMING_HIGH : Field.HOMING_LOW)); 
            addBlock(followPath(
                () -> perception.generateHomingPosition(homingPose)
            ));

            // TODO: evaluate fail conditions
            addBlock(new TorqueWaitUntil(() -> {
                final Optional<Note> noteOpt = perception.getBestDetection();
                if (noteOpt.isPresent()) {
                    bestNote = noteOpt.get();
                    return true;
                }
                return false;
            }));

            addBlock(shooter.yieldState(Shooter.State.SMART));

            addBlock(new TorqueRun(() -> drivebase.setAlignTargetRelative(Rotation2d.fromDegrees(bestNote.angle))));
            addBlock(drivebase.yieldState(Drivebase.State.ALIGN_TO_ANGLE));
            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));
            addBlock(drivebase.yieldState(Drivebase.State.ROBOT_RELATIVE));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(new TorqueSwerveSpeeds(0, 0, 0))));
            addBlock(new TorqueWaitUntil(this::stopIntaking));
            addBlock(shooter.yieldState(Shooter.State.WARMUP));

            addBlock(new TorqueRun(
                    () -> shootingPose = perception.isAboveSpeakerOnY() ? Field.SHOOT_HIGH : Field.SHOOT_LOW));
            // and the above line will also pass
            addBlock(followPath(() -> perception.generateShootingPosition(shootingPose)));

            addBlock(new TorqueRunSequence(new Shoot()));
        }
    }

    private AutoConfig config;

    public DynamicAuto() {
        config = getConfigFromNT();

        // This is debug only
        // addBlock(new TorqueRun(() -> perception.setPose(new Pose2d(1.1, 5.75, Field.ROT_FWD))));

        // addBlock(shooter.yieldState(Shooter.State.WARMUP));
        addBlock(new TorqueRunSequence(new Shoot()));

        addBlock(intake.yieldState(Intake.State.SMART_INTAKE));

        addBlock(followPath(() -> perception.generateInitial(config.getNextNote())));

        addBlock(new TorqueWaitUntil(shooter::hasGateSpiked));

        addBlock(new TorqueRunSequence(new Shoot()));

        addBlock(new TorqueWhile(config::hasNext, new GetAndScoreCloseNote(config)));

        addBlock(new TorqueSwitch(config::isDoingCenter, new HandleCenterLineNotes()));
    }

    public AutoConfig getConfigFromNT() {
        // final String command = Debug.getAutoCommandEntry().getString("");
        final String command = "21";

        final List<Integer> notes = new ArrayList<>();
        for (int i = 0; i < command.length(); i++) {
            char ch = command.charAt(i);
            if (ch >= '1' && ch <= '3')
                notes.add(ch - '0');
        }
        final boolean doingCenter = !command.contains("X");
        return new AutoConfig(notes, doingCenter);
    }

    public class AutoConfig {

        private final List<Integer> allNotes;
        private final List<Integer> notes;
        private final boolean doCenter;

        public final boolean isDoingCenter() {
            return doCenter;
        }

        public AutoConfig(List<Integer> notes, boolean doCenter) {
            this.allNotes = notes;
            this.doCenter = doCenter;
            this.notes = new ArrayList<>();
            reset();
        }

        public int getNextNote() {
            System.out.println(notes.get(0));
            return notes.remove(0);
        }

        public boolean hasNext() {
            return notes.size() > 0;
        }

        public void reset() {
            notes.clear();
            for (int note : allNotes) {
                notes.add(note);
            }
        }

    }
}