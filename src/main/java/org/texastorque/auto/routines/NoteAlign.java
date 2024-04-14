package org.texastorque.auto.routines;

import java.util.function.BooleanSupplier;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Shooter.GateState;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * A sequence for deploying the intake after some boolean condition,
 * provided with a BooleanSupplier yields true.
 */
public class NoteAlign extends TorqueSequence implements Subsystems {

    private boolean use = false;

    public NoteAlign(final BooleanSupplier useSupplier, final BooleanSupplier when) {
        addBlock(new TorqueRun(() -> use = useSupplier.getAsBoolean()));

        addBlock(new TorqueWaitUntil(() -> drivebase.getChassisSpeeds().vxMetersPerSecond > 0.1));

        // addBlock(new TorqueWaitUntil(() ->  !use || field.isXPast(perception.getPose(), 6)));
        addBlock(new TorqueWaitUntil(when));

        addBlock(new TorqueRun(() -> perception.useDetectionLock(use)));

        addBlock(new TorqueWaitUntil(() -> drivebase.getChassisSpeeds().vxMetersPerSecond < 0.1));

        addBlock(new TorqueRun(() -> perception.useDetectionLock(false)));
    }
}