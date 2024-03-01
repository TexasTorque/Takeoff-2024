package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    public static volatile Climber instance;

    private static final double CLIMB_VOLTS = 12;
    private static final double TRAP_VOLTS = 1;
    // private static final double TRAP_POSITION = 0;
    private static final double TRAP_TOLERANCE = .3;

    private final TorqueNEO left, right;
    private final TorqueNEO hook;

    private double leftTare, rightTare;
    private double leftPosition, rightPosition;

    private HookState hookState = HookState.OFF;

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

    public static enum HookState implements TorqueState {
        IN(TRAP_VOLTS), OUT(-TRAP_VOLTS), OFF(0), IDLE(-TRAP_VOLTS / 3), TRAP, HOLD;

        private final double volts;

        private HookState(final double volts) {
            this.volts = volts;
        }

        private HookState() {
            volts = 0;
        }
    }

    private PIDController hookPID;

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

        hook = new TorqueNEO(Ports.HOOK);
        hook.setVoltageCompensation(12.6);
        hook.setCurrentLimit(25);
        hook.setBreakMode(true);
        hook.invertMotor(true);
        hook.burnFlash();

        hookPID = new PIDController(1, 0, 0);

        SmartDashboard.putNumber("Hook P", 1);
        SmartDashboard.putNumber("Hook I", 1);
        SmartDashboard.putNumber("Hook D", 1);

        SmartDashboard.putNumber("Hold Voltage", -1.25);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    public boolean isReady() {
        return TorqueMath.toleranced(perception.getGyroPitch(), 7.2, 1);
    }

    @Override
    public void update(TorqueMode mode) {
        leftPosition = left.getPosition() - leftTare;
        rightPosition = right.getPosition() - rightTare;

        hookPID.setP(SmartDashboard.getNumber("Hook P", 0));
        hookPID.setI(SmartDashboard.getNumber("Hook I", 0));
        hookPID.setD(SmartDashboard.getNumber("Hook D", 0));

        if (!shooter.wantsToClimb() && !shooter.inDebugMode()) {
            desiredState = State.OFF;
            hookState = HookState.IDLE;
        }

        Debug.log("Trap State", hookState.toString());

        Debug.log("Climber Left", leftPosition);
        Debug.log("Climber Right", rightPosition);

        Debug.log("Is Ready", isReady());

        double leftSpeed = desiredState.leftVolts;
        double rightSpeed = desiredState.rightVolts;
        double hookSpeed = hookState.volts;

        if (hookState == HookState.TRAP) {
            hookSpeed = TorqueMath.constrain(hookPID.calculate(perception.getGyroPitch(), 8.75),
                    SmartDashboard.getNumber("Hook Max Volts", 0));
        } else if (hookState == HookState.HOLD && (shooter.wantsState(Shooter.State.CLIMB) || shooter.wantsState(Shooter.State.TRAP)) ) {
            // hookSpeed = SmartDashboard.getNumber("Hold Voltage", 0);
            hookSpeed = -1.25;

        }

        // if (shooter.inDebugMode()) {
        // leftSpeed /= 3;
        // rightSpeed /= 3;
        // trapSpeed /= 2;
        // }

        hook.setVolts(hookSpeed);
        left.setVolts(leftSpeed);
        right.setVolts(rightSpeed);
    }

    public void tareClimber() {
        leftTare = left.getPosition();
        rightTare = right.getPosition();
    }

    @Override
    public void clean(TorqueMode mode) {
        desiredState = State.OFF;
        hookState = HookState.OFF;
    }

    public void setHookState(HookState trapState) {
        this.hookState = trapState;
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
