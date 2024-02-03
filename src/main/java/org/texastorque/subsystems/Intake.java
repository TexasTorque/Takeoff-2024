package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    private static volatile Intake instance;

    private static final double ROTARY_DOWN = 14;

    public static enum State implements TorqueState {
        OFF(0, 0), INTAKE(ROTARY_DOWN, 12),
        SMART_INTAKE(ROTARY_DOWN, 12), OUTTAKE(ROTARY_DOWN, -12);

        public final double rotaryPosition, rollerSpeed;

        private State(final double rotaryPosition, final double rollerSpeed) {
            this.rotaryPosition = rotaryPosition;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private static final double ROTARY_TOLERANCE = 5;

    private final TorqueNEO rotary, rollers;

    private final TorqueRequestableTimeout spikeTimeout;

    private boolean spiked = false;

    public Intake() {
        super(State.OFF);

        rotary = new TorqueNEO(Ports.INTAKE_ROTARY_LEFT);
        rotary.addFollower(Ports.INTAKE_ROTARY_RIGHT, false);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.invertMotor(true);
        rotary.setPIDFeedbackDevice(rotary.encoder);
        rotary.configurePIDF(.05, 0, 0, 0);
        rotary.burnFlash();

        rollers = new TorqueNEO(Ports.INTAKE_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setBreakMode(false);
        rollers.invertMotor(true);
        rollers.burnFlash();

        spikeTimeout = new TorqueRequestableTimeout();
    }

    @Override
    public void initialize(final TorqueMode mode) {
    }

    public boolean isRotaryAtState() {
        return TorqueMath.toleranced(Math.abs(rotary.getPosition()), desiredState.rotaryPosition, ROTARY_TOLERANCE);
    }

    public boolean isCurrentSpike() {
        return spiked;
    }

    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Intake State", desiredState.toString());
        Debug.log("Intake Rotary", rotary.getPosition());

        if (wantsState(State.SMART_INTAKE)) {
            if (!spikeTimeout.get() && shooter.hasGateSpiked()) {
                Input.getInstance().setRumbleFor(.2);
                spiked = true;
            }
        } else {
            spikeTimeout.set(1);
            spiked = false;
        }

        if (spiked && shooter.isRotaryAtState())
            desiredState = State.OFF;

        rollers.setVolts(desiredState.rollerSpeed);
        rotary.setPosition(desiredState.rotaryPosition);

    }

    public boolean isIntaking() {
        return wantsState(State.INTAKE) || wantsState(State.SMART_INTAKE);
    }

    public boolean isOutaking() {
        return wantsState(State.OUTTAKE);
    }

    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = State.OFF;
        }
    }
}