package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    private static volatile Climber instance;

    public static enum State implements TorqueState {
        UP(40), DOWN(0);

        private final double position;

        private State(final double position) {
            this.position = position;
        }
    }

    public static enum HookState implements TorqueState {
        OUT(20), IN(0);

        private final double position;

        private HookState(final double position) {
            this.position = position;
        }
    }

    private HookState desiredHookState = HookState.IN;

    private final TorqueNEO winch, hook;

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
    public void initialize(final TorqueMode mode) {
        
    }

    @Override
    public void update(final TorqueMode mode) {
        winch.setPosition(desiredState.position);
        hook.setPosition(desiredHookState.position);
    }

    public void setHookState(HookState hookState) {
        this.desiredHookState = hookState;
    }

    public static synchronized final Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
