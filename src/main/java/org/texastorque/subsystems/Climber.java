package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    public static volatile Climber instance;

    private static final double CLIMB_VOLTS = 12;

    private final TorqueNEO left, right;

    public static enum State implements TorqueState {
        OFF(0, 0),
        UP(CLIMB_VOLTS, CLIMB_VOLTS),
        DOWN(-CLIMB_VOLTS, -CLIMB_VOLTS),
        LEFT_UP(CLIMB_VOLTS, 0),
        RIGHT_UP(0, CLIMB_VOLTS),
        LEFT_DOWN(-CLIMB_VOLTS, 0),
        RIGHT_DOWN(0, -CLIMB_VOLTS);

        public final double leftVolts, rightVolts;

        private State(final double leftVolts, final double rightVolts) {
            this.leftVolts = leftVolts;
            this.rightVolts = rightVolts;
        }
    }

    protected Climber() {
        super(State.OFF);

        left = new TorqueNEO(Ports.CLIMBER_LEFT);
        left.setVoltageCompensation(12.6);
        left.setCurrentLimit(80);
        left.setBreakMode(true);
        left.invertMotor(false);
        left.burnFlash();

        right = new TorqueNEO(Ports.CLIMBER_RIGHT);
        right.setVoltageCompensation(12.6);
        right.setCurrentLimit(80);
        right.setBreakMode(true);
        right.invertMotor(true);
        right.burnFlash();
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        if (!shooter.wantsToClimb() && !shooter.inDebugMode() && !shooter.wantsState(Shooter.State.AMP)) {
            desiredState = State.OFF;
        }

        double leftSpeed = desiredState.leftVolts;
        double rightSpeed = desiredState.rightVolts;

        left.setVolts(leftSpeed);
        right.setVolts(rightSpeed);
    }

    @Override
    public void clean(TorqueMode mode) {
        desiredState = State.OFF;
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
