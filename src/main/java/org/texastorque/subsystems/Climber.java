package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    private static volatile Climber instance;

    public static enum State implements TorqueState {
        UP(40, 0), DOWN(0,0), TRAP(0, 20);

        private final double climberPosition, hookPosition;

        private State(final double climberPosition, final double hookPosition) {
            this.climberPosition = climberPosition;
            this.hookPosition = hookPosition;
        }
    }

    private final TorqueNEO winch, hook;

    private final double WINCH_TOLERANCE = 5;

    public Climber() {
        super(State.DOWN);

        winch = new TorqueNEO(Ports.CLIMBER_LEFT);
        winch.addFollower(Ports.CLIMBER_RIGHT, true);
        winch.setVoltageCompensation(12.6);
        winch.setBreakMode(true);
        winch.setPIDFeedbackDevice(winch.encoder);
        winch.configurePIDF(1, 0, 0, 0);
        winch.burnFlash();

        hook = new TorqueNEO(Ports.HOOK);
        hook.setVoltageCompensation(12.6);
        hook.setBreakMode(true);
        hook.setPIDFeedbackDevice(hook.encoder);
        hook.configurePIDF(1, 0, 0, 0);
        hook.burnFlash();
    }

    @Override
    public void initialize(final TorqueMode mode) {}

    public boolean isWinchAtState() {
        return TorqueMath.toleranced(winch.getPosition(), desiredState.climberPosition, WINCH_TOLERANCE);
    }

    @Override
    public void update(final TorqueMode mode) {
        winch.setPosition(desiredState.climberPosition);
        if (isWinchAtState()) hook.setPosition(desiredState.hookPosition);
    }

    public static synchronized final Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}