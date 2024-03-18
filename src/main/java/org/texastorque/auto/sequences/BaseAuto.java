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
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWhile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/** Trivial "Base" autos (going from one note to another while shooting) */
public class BaseAuto extends TorqueSequence implements Subsystems {

    private Timer totalAutoTimer = new Timer();

    private final NoteSequence noteSequence;

    /** Non-auto */
    public BaseAuto(final Pose2d initPose) {
        noteSequence = null;
        addBlock(new TorqueRun(() -> perception.resetPose(field.getAllianceReflectedPose(initPose))));

        addBlock(new TorqueRunSequence(new Shoot(Shooter.State.LAYUP)));
    }

    /** Pass in the order of notes */
    public BaseAuto(final Pose2d initPose, final int... notes) {
        noteSequence = new NoteSequence(false, notes);

        addBlock(new TorqueRun(() -> perception.resetPose(field.getAllianceReflectedPose(initPose))));

        addBlock(new TorqueRun(() -> totalAutoTimer.restart()));

        addBlock(new TorqueRun(() -> perception.setFutureShootingPose(perception.getPose())));


        addBlock(new TorqueRunSequence(new Shoot(Shooter.State.LAYUP)));

        addBlock(new TorqueWhile(noteSequence::hasNext, new CollectAndShootNote(noteSequence)));
    }

}