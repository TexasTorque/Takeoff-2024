package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public class Climber extends TorqueStatorSubsystem<Climber.State> implements Subsystems {
    public static volatile Climber instance;

    private static final double CLIMB_VOLTS = 12;

    private static final double CLIMBER_MIN = -10, CLIMBER_MAX = 10; // zero'd from mid

    private static final double MIN_ANGLE = 2 /* deg */, kV = 0.02; /* V/deg */

    private final TorqueNEO left, right;

    private double leftTare, rightTare;
    private double leftPosition, rightPosition;



    public static enum State implements TorqueState {
        OFF(0, 0),
        UP(CLIMB_VOLTS, CLIMB_VOLTS),
        DOWN(-CLIMB_VOLTS, -CLIMB_VOLTS),
        LEFT_UP(CLIMB_VOLTS, 0),
        RIGHT_UP(0, CLIMB_VOLTS),
        LEFT_DOWN(-CLIMB_VOLTS, 0),
        RIGHT_DOWN(0, -CLIMB_VOLTS),
        BALANCE_UP(CLIMB_VOLTS, CLIMB_VOLTS);

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
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        leftPosition = left.getPosition() - leftTare;
        rightPosition = right.getPosition() - rightTare;

        Debug.log("Left Climb Position", leftPosition);
        Debug.log("Right Climb Position", rightPosition);
        Debug.log("Climb State", desiredState.toString());

        double roll = perception.getRoll().getDegrees();
        // deadbands the angle to call angles close to zero effectively zero
        roll = TorqueMath.scaledLinearDeadband(roll, MIN_ANGLE); 

        if (!shooter.wantsToClimb() && !shooter.inDebugMode())
            desiredState = State.OFF;

        double leftSpeed = desiredState.leftVolts;
        double rightSpeed = desiredState.rightVolts;

        if (wantsState(State.BALANCE_UP)) {
            // v = Vmax(1±kθ, 1)
            leftSpeed = leftSpeed * Math.max(1 + kV * roll, 1);
            rightSpeed = rightSpeed * Math.max(1 - kV * roll, 1);

            // Examples:
            // - given roll=5° kV=.02 v_l=v_r=12
            //   v_l = 12 * max(1 + .02*5, 1) = 12 * 1 = 12v
            //   v_r = 12 * max(1 - .02*5, 1) = 12 * 0.9 = 10.8v
            // - given roll=-10° kV=.02 v_l=v_r=12
            //   v_l = 12 * max(1 + .02*-10, 1) = 12 * 0.8 = 9.6v
            //   v_r = 12 * max(1 - .02*-10, 1) = 12 * 1 = 12v
        }

        if (shooter.inDebugMode()) {
            leftSpeed /= 3;
            rightSpeed /= 3;
        } else {
            // Comenting out the linear constraint for now, unecesary 
            // leftSpeed = TorqueMath.linearConstraint(leftSpeed, leftPosition, CLIMBER_MIN, CLIMBER_MAX);
            // rightSpeed = TorqueMath.linearConstraint(rightSpeed, rightPosition, CLIMBER_MIN, CLIMBER_MAX);
        }

        left.setVolts(leftSpeed);
        right.setVolts(rightSpeed);
    }

    @Override
    public void clean(TorqueMode mode) {
        desiredState = State.OFF;
    }

    public void tareClimber() {
        leftTare = left.getPosition();
        rightTare = right.getPosition();
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
