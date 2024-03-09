package org.texastorque.auto.commands;

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
public class DeployIntakeWhen extends TorqueSequence implements Subsystems {

    private Timer timer = new Timer();
    private boolean isCenterLine = true;

    public DeployIntakeWhen(final BooleanSupplier when) {
        addBlock(new TorqueRun(() -> isCenterLine = !when.getAsBoolean()));
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

        addBlock(new TorqueRun(() -> perception.setFutureShootingPose(
                !isCenterLine ? TorqueFollowPath.getEndingPositionForCurrentlyLoadedPath()
                        : field.calculateXOffset(TorqueFollowPath.getEndingPositionForCurrentlyLoadedPath(), .5)

        )));

        addBlock(shooter.yieldState(Shooter.State.FUTURE_SMART));

        addBlock(intake.yieldState(Intake.State.AUTO_PRIME));

    }
}