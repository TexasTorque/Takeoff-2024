package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import edu.wpi.first.math.controller.PIDController;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    private static volatile Intake instance;

    private final static double ROTARY_DOWN = 13;

    public static enum State implements TorqueState {
        OFF(0, 0), INTAKE(ROTARY_DOWN, 10),
        SMART_INTAKE(ROTARY_DOWN, 10), OUTTAKE(ROTARY_DOWN, -10), AUTO_PRIME(7, 0), PRIME(4, 0);

        public final double rotaryPosition, rollerSpeed;

        private State(final double rotaryPosition, final double rollerSpeed) {
            this.rotaryPosition = rotaryPosition;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private static final double ROTARY_TOLERANCE = 4;

    private final TorqueNEO rotaryLeft, rotaryRight, rollers;

    private final PIDController rotaryLeftPID, rotaryRightPID;

    public Intake() {
        super(State.OFF);

        rotaryLeft = new TorqueNEO(Ports.INTAKE_ROTARY_LEFT);
        rotaryLeft.setVoltageCompensation(12.6);
        rotaryLeft.setCurrentLimit(25);
        rotaryLeft.setBreakMode(true);
        rotaryLeft.invertMotor(false);
        rotaryLeft.setPIDFeedbackDevice(rotaryLeft.encoder);
        rotaryLeft.burnFlash();

        rotaryLeftPID = new PIDController(.5, 0, 0);

        rotaryRight = new TorqueNEO(Ports.INTAKE_ROTARY_RIGHT);
        rotaryRight.setVoltageCompensation(12.6);
        rotaryRight.setCurrentLimit(25);
        rotaryRight.setBreakMode(true);
        rotaryRight.invertMotor(true);
        rotaryRight.setPIDFeedbackDevice(rotaryRight.encoder);
        rotaryRight.burnFlash();

        rotaryRightPID = new PIDController(.5, 0, 0);

        rollers = new TorqueNEO(Ports.INTAKE_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setCurrentLimit(25);
        rollers.setBreakMode(false);
        rollers.invertMotor(true);
        rollers.burnFlash();
    }

    @Override
    public void initialize(final TorqueMode mode) {
    }

    public boolean isAtState() {
        return isIntaking() && Math.abs(
                Math.abs(rotaryLeft.getPosition()) - Math.abs(desiredState.rotaryPosition)) <= ROTARY_TOLERANCE;
    }

    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Intake State", desiredState.toString());
        Debug.log("Intake Rotary Right", rotaryRight.getPosition());
        Debug.log("Intake Rotary Left", rotaryLeft.getPosition());
        Debug.log("Rotary Down Enough", isAtState());

        if (wantsState(State.SMART_INTAKE) && shooter.hasNote()) {
            Input.getInstance().setRumbleFor(.2);

            if (shooter.isRotaryAtState())
                desiredState = mode.isAuto() ? State.AUTO_PRIME : State.OFF;

        } else if (!isIntaking() && !isOutaking() && shooter.isShift()) {
            desiredState = State.PRIME;
        }

        rollers.setVolts(desiredState.rollerSpeed);

        rotaryLeft.setVolts(rotaryLeftPID.calculate(rotaryLeft.getPosition(),
                desiredState.rotaryPosition));
        rotaryRight.setVolts(rotaryRightPID.calculate(rotaryLeft.getPosition(),
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