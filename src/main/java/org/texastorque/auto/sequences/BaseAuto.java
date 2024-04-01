/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import java.util.ArrayList;
import java.util.List;

import org.texastorque.Subsystems;
import org.texastorque.auto.AutoManager;
import org.texastorque.auto.NoteSequence;
import org.texastorque.auto.NoteSequence.Location;
import org.texastorque.auto.routines.CollectAndShootNote;
import org.texastorque.auto.routines.Shoot;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueWhile;

/** Trivial "Base" autos (going from one note to another while shooting) */
public class BaseAuto extends TorqueSequence implements Subsystems {

    private final NoteSequence noteSequence;

    /** Pass in the sequence of notes */
    public BaseAuto(final String extraTimePathName, final NoteSequence ns) {
        this.noteSequence = ns;

        // Reset the pose to the initial position for this sequence
        addBlock(new TorqueRun(() -> perception.resetPose(
            field.getAllianceReflectedPose(noteSequence.peekNext().getStartingPose()))));

        // I think this is redundent -- double check pls!
        addBlock(new TorqueRun(() -> perception.setFutureShootingPose(perception.getPose())));

        // Take the first shot as a layup -- this will change if starting point isnt on subwoofer
        addBlock(new TorqueRunSequence(new Shoot(Shooter.State.LAYUP)));

        addBlock(new TorqueWhile(noteSequence::hasNext, new CollectAndShootNote(noteSequence)));

        // addBlock(new TorqueFollowPath(() -> AutoManager.getInstance().getPath(extraTimePathName), drivebase));
    }

   
}