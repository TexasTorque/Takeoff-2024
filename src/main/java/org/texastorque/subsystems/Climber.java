package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    public static volatile Climber instance;

    private static final double CLIMB_VOLTS = 12;
    private static final double TRAP_VOLTS = 2;

    private final TorqueNEO left, right;
    private final TorqueNEO trap;

    private TrapState trapState = TrapState.OFF;

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

    public static enum TrapState implements TorqueState {
        IN(TRAP_VOLTS), OUT(-TRAP_VOLTS), OFF(0), IDLE(-TRAP_VOLTS / 6);

        private final double volts;

        private TrapState(final double volts) {
            this.volts = volts;
        }
    }

    protected Climber() {
        super(State.OFF);

        left = new TorqueNEO(Ports.CLIMBER_LEFT);
        left.setVoltageCompensation(12.6);
        left.setCurrentLimit(25);
        left.setBreakMode(true);
        left.invertMotor(false);
        left.burnFlash();

        right = new TorqueNEO(Ports.CLIMBER_RIGHT);
        right.setVoltageCompensation(12.6);
        right.setCurrentLimit(25);
        right.setBreakMode(true);
        right.invertMotor(true);
        right.burnFlash();

        trap = new TorqueNEO(Ports.HOOK);
        trap.setVoltageCompensation(12.6);
        trap.setCurrentLimit(25);
        trap.setBreakMode(true);
        trap.invertMotor(true);
        trap.burnFlash();
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        if (!shooter.wantsToClimb() && !shooter.inDebugMode()) {
            desiredState = State.OFF;
            trapState = TrapState.IDLE;
        }

        Debug.log("Trap State", trapState.toString());

        double leftSpeed = desiredState.leftVolts;
        double rightSpeed = desiredState.rightVolts;
        double trapSpeed = trapState.volts;

        if (shooter.inDebugMode()) {
            leftSpeed /= 3;
            rightSpeed /= 3;
            trapSpeed /= 2;
        } else if (drivebase.isDecelerating()) {
            leftSpeed /= 6;
            rightSpeed /= 6;
        }

        trap.setVolts(trapSpeed);
        left.setVolts(leftSpeed);
        right.setVolts(rightSpeed);
    }

    @Override
    public void clean(TorqueMode mode) {
        desiredState = State.OFF;
        trapState = TrapState.OFF;
    }

    public void setTrapState(TrapState trapState) {
        this.trapState = trapState;
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
