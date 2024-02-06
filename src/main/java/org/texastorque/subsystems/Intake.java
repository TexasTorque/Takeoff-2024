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
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    private static volatile Intake instance;

    public static enum State implements TorqueState {
        OFF(0, 0, 0), INTAKE(11, 7.1, 12),
        SMART_INTAKE(11, 7.1, 12), OUTTAKE(11, 7.1, -12);

        public final double rotaryLeftPosition, rotaryRightPosition, rollerSpeed;

        private State(final double rotaryLeftPosition, final double rotaryRightPosition, final double rollerSpeed) {
            this.rotaryLeftPosition = rotaryLeftPosition;
            this.rotaryRightPosition = rotaryRightPosition;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private static final double ROTARY_TOLERANCE = 5;

    private final TorqueNEO rotaryLeft, rotaryRight, rollers;

    private final PIDController rotaryLeftPID, rotaryRightPID;

    private final TorqueRequestableTimeout spikeTimeout;

    private boolean spiked = false;

    public Intake() {
        super(State.OFF);

        rotaryLeft = new TorqueNEO(Ports.INTAKE_ROTARY_LEFT);
        rotaryLeft.setVoltageCompensation(12.6);
        rotaryLeft.setBreakMode(false);
        rotaryLeft.invertMotor(false);
        rotaryLeft.setPIDFeedbackDevice(rotaryLeft.encoder);
        rotaryLeft.burnFlash();

        rotaryLeftPID = new PIDController(.1, 0, 0);

        rotaryRight = new TorqueNEO(Ports.INTAKE_ROTARY_RIGHT);
        rotaryRight.setVoltageCompensation(12.6);
        rotaryRight.setBreakMode(false);
        rotaryRight.invertMotor(true);
        rotaryRight.setPIDFeedbackDevice(rotaryRight.encoder);
        rotaryRight.burnFlash();

        rotaryRightPID = new PIDController(.1, 0, 0);

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
        return TorqueMath.toleranced(Math.abs(rotaryLeft.getPosition()), desiredState.rotaryLeftPosition,
                ROTARY_TOLERANCE);
    }

    public boolean isCurrentSpike() {
        return spiked;
    }

    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Intake State", desiredState.toString());
        Debug.log("Intake Rotary Left", rotaryLeft.getPosition());
        Debug.log("Intake Rotary Right", rotaryRight.getPosition());

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

        // rotaryLeft.setVolts(rotaryLeftPID.calculate(rotaryLeft.getPosition(), desiredState.rotaryLeftPosition));
        // rotaryRight.setVolts(rotaryRightPID.calculate(rotaryRight.getPosition(), desiredState.rotaryRightPosition));
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