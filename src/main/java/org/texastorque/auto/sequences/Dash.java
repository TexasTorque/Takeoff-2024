/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.CollectAndShootNote;
import org.texastorque.auto.commands.NoteSequence;
import org.texastorque.auto.commands.Shoot;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Shooter.GateState;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/** Auto for racing to the center line */
public class Dash extends TorqueSequence implements Subsystems {

    private final NoteSequence noteSequence;
    private final Timer timer = new Timer();

    public Dash(final int... notes) {
        noteSequence = new NoteSequence(true, notes);

        addBlock(new TorqueRun(() -> noteSequence.popNote()));

        addBlock(new TorqueRun(() -> timer.start()));
        addBlock(new TorqueRun(() -> perception.resetPose(new Pose2d(1.38, 7.4, new Rotation2d(0)))));

        addBlock(drivebase.yieldState(Drivebase.State.DASH));

        // deploy intake after some ammount of seconds elapsed
        addBlock(new TorqueWaitUntil(() -> timer.get() >= 1.25));

        addBlock(intake.yieldState(Intake.State.SMART_INTAKE));
        addBlock(new TorqueWaitUntil(intake::isAtState));
        addBlock(shooter.yieldState(Shooter.State.INTAKE));
        addBlock(shooter.yieldGateState(Shooter.GateState.IN));
  
        // addBlock(new TorqueWaitUntil(() -> field.isXPast(perception.getPose(),
        // 6.5))); // give up trying to drive further
         
        // reverse course after some ammount of seconds elapsed
  
        addBlock(new TorqueWaitUntil(() -> timer.get() >= 1.95)); // Hit the center line

        addBlock(new TorqueRun(() -> timer.restart()));


        addBlock(new TorqueRun(() -> perception.resetPose(new Pose2d(9, 7.44, new Rotation2d(0)))));

        // addBlock(drivebase.yieldState(Drivebase.State.FIELD_RELATIVE)); // probs don't need this bc of path following
        // addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(new TorqueSwerveSpeeds(0, 0, 0))));

        // start pathing...
        addBlock(new TorqueFollowPath("go_10_to_shoot", drivebase));
        // ...and keep trying to intake until it has note.

        addBlock(new TorqueWaitUntil(() -> (shooter.hasNote() || timer.get() >= 3))); // start pathing and keep trying to intake until it has note.

        addBlock(shooter.yieldGateState(GateState.OFF));
        addBlock(new TorqueRun(() -> shooter.setState(Shooter.State.FUTURE_SMART)));

        addBlock(new TorqueRun(() -> perception.setFutureShootingPose(
        // : field.calculateXOffset(TorqueFollowPath.getEndingPositionForCurrentlyLoadedPath(), .35)
            field.calculateXOffset(TorqueFollowPath.getEndingPositionForCurrentlyLoadedPath(), 0)

    )));

        
        addBlock(new TorqueRunSequence(new Shoot()));


        addBlock(new TorqueWhile(noteSequence::hasNext, new CollectAndShootNote(noteSequence)));
    }
}
