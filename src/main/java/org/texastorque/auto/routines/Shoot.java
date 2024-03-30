package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Shooter;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This should shoot the gamepeice using smartshot, therefor aligning drivebase,
 * and will wait until the shooter is ready
 */
public class Shoot extends TorqueSequence implements Subsystems {


    public Shoot() {
        this(Shooter.State.SMART);
    }

    public Shoot(final Shooter.State shooterState) {
        log("Auto State", () -> "SHOOTING");

        addBlock(shooter.yieldState(shooterState));

        if (RobotBase.isReal()) {
            addBlock(new TorqueWaitUntil(() -> !shooter.hasNote()));
        } else {
            addBlock(new TorqueWaitTime(1));
        }
        log("Auto State", () -> "GOT NOTE");

        addBlock(shooter.yieldState(Shooter.State.AUTO_OFF));
        addBlock(shooter.yieldGateState(Shooter.GateState.OFF));
        log("Auto State", () -> "SHOT");
    }
}
