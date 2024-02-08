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
import edu.wpi.first.math.controller.PIDController;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    private static volatile Intake instance;

    private final static double ROTARY_DOWN = 12;

    public static enum State implements TorqueState {
        OFF(0, 0), INTAKE(ROTARY_DOWN, 12),
        SMART_INTAKE(ROTARY_DOWN, 12), OUTTAKE(ROTARY_DOWN, -12);

        public final double rotaryPosition, rollerSpeed;

        private State(final double rotaryPosition, final double rollerSpeed) {
            this.rotaryPosition = rotaryPosition;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private static final double ROTARY_TOLERANCE = 7;

    private final TorqueNEO rotaryLeft, rotaryRight, rollers;

    private final PIDController rotaryLeftPID, rotaryRightPID;

    private final TorqueRequestableTimeout spikeTimeout;

    private boolean spiked = false;

    public Intake() {
        super(State.OFF);

        rotaryLeft = new TorqueNEO(Ports.INTAKE_ROTARY_LEFT);
        rotaryLeft.setVoltageCompensation(12.6);
        rotaryLeft.setBreakMode(true);
        rotaryLeft.invertMotor(false);
        rotaryLeft.setPIDFeedbackDevice(rotaryLeft.encoder);
        rotaryLeft.burnFlash();

        rotaryLeftPID = new PIDController(.5, 0, 0);

        rotaryRight = new TorqueNEO(Ports.INTAKE_ROTARY_RIGHT);
        rotaryRight.setVoltageCompensation(12.6);
        rotaryRight.setBreakMode(true);
        rotaryRight.invertMotor(true);
        rotaryRight.setPIDFeedbackDevice(rotaryRight.encoder);
        rotaryRight.burnFlash();

        rotaryRightPID = new PIDController(.5, 0, 0);

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

    public boolean isRotaryDownEnough() {
        return TorqueMath.toleranced(Math.abs(rotaryLeft.getPosition()), desiredState.rotaryPosition,
                ROTARY_TOLERANCE)
                && TorqueMath.toleranced(Math.abs(rotaryRight.getPosition()), desiredState.rotaryPosition);
    }

    public boolean isCurrentSpike() {
        return spiked;
    }

    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Intake State", desiredState.toString());
        Debug.log("Intake Rotary Left", rotaryLeft.getPosition());
        Debug.log("Intake Rotary Right", rotaryRight.getPosition());

        if (wantsState(State.SMART_INTAKE) && !spiked) {
            if (!spikeTimeout.get() && shooter.hasGateSpiked()) {
                Input.getInstance().setRumbleFor(.2);
                spiked = true;
            }
        } else {
            spikeTimeout.set(.5);
            spiked = false;
        }

        if (spiked && shooter.isRotaryAtState())
            desiredState = State.OFF;

        Debug.log("Desired State After", desiredState.toString());
        Debug.log("spiked", spiked);

        rollers.setVolts(desiredState.rollerSpeed);

        rotaryLeft.setVolts(rotaryLeftPID.calculate(rotaryLeft.getPosition(),
                desiredState.rotaryPosition));
        rotaryRight.setVolts(rotaryRightPID.calculate(rotaryRight.getPosition(),
                desiredState.rotaryPosition));
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = State.OFF;
        }
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
}