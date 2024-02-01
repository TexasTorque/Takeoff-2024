package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueLookUpTable;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter extends TorqueStatorSubsystem<Shooter.State> implements Subsystems {

    private static volatile Shooter instance;

    public static record Shot(double topVelocity, double bottomVelocity, double angle) {
        public Shot(double velocity, double angle) {
            this(velocity, velocity, angle);
        }

        private static final Shot empty = new Shot(0, 0, 0);
    }

    public static enum State implements TorqueState {
        OFF(false),
        INTAKE(new Shot(1477, 1477), false),
        AMP(new Shot(1477, 1477), true),
        TRAP(new Shot(1477, 1477), false),
        SMART(true),
        WARMUP(new Shot(1477, 1477), false),
        LAYUP(new Shot(1477, 1477), true),
        SAFEZONE(new Shot(1477, 1477), true);

        public final Shot shot;
        public final boolean allowedToShoot;

        private State(final boolean allowedToShoot) {
            this(Shot.empty, allowedToShoot);
        }

        private State(final Shot shot, final boolean allowedToShoot) {
            this.shot = shot;
            this.allowedToShoot = allowedToShoot;
        }
    }

    public static enum GateState implements TorqueState {
        OFF(0), IN(-12), OUT(12);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    private static final double FLYWHEEL_TOLERANCE = 100, ROTARY_TOLERANCE = 5, GATE_CURRENT_SPIKE = 10;

    private final TorqueNEO rotary, flywheelTop, flywheelBottom, gate;

    private final CANcoder rotaryEncoder, flywheelTopEncoder, flywheelBottomEncoder;

    private final PIDController rotaryPID, flywheelPID;
    private final SimpleMotorFeedforward flywheelFF;

    private final TorqueLookUpTable<Shot> shotTable;

    private GateState gateState = GateState.OFF;

    public static double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }

    public Shooter() {
        super(State.OFF);

        rotary = new TorqueNEO(Ports.SHOOTER_ROTARY);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        flywheelTop = new TorqueNEO(Ports.FLYWHEEL_TOP);
        flywheelTop.setVoltageCompensation(12.6);
        flywheelTop.setBreakMode(false);
        flywheelTop.burnFlash();

        flywheelTopEncoder = new CANcoder(Ports.FLYWHEEL_TOP_ENCODER);
        flywheelBottomEncoder = new CANcoder(Ports.FLYWHEEL_BOTTOM_ENCODER);

        flywheelBottom = new TorqueNEO(Ports.FLYWHEEL_BOTTOM);
        flywheelBottom.setVoltageCompensation(12.6);
        flywheelBottom.setBreakMode(false);
        flywheelBottom.burnFlash();

        flywheelPID = new PIDController(1, 0, 0);
        flywheelFF = new SimpleMotorFeedforward(0, 0, 0);

        gate = new TorqueNEO(Ports.SHOOTER_GATE);
        gate.setVoltageCompensation(12.6);
        gate.setBreakMode(true);

        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);
        rotaryPID = new PIDController(1, 0, 0);

        shotTable = new TorqueLookUpTable<Shot>(
                (final Shot me, final Shot other) -> Math.abs(other.angle - me.angle) < 0.1
                        && Math.abs(other.topVelocity - me.topVelocity) < 0.1,
                (final Shot me, final Shot end, final Double t) -> new Shot(lerp(me.topVelocity, end.topVelocity, t),
                        lerp(me.angle, end.angle, t)));
    }

    public boolean hasGateSpiked() {
        return gate.getCurrent() >= GATE_CURRENT_SPIKE;
    }

    public boolean isReadyToShoot() {
        return TorqueMath.toleranced(flywheelTop.getVelocity(), desiredState.shot.topVelocity, FLYWHEEL_TOLERANCE)
                && TorqueMath.toleranced(flywheelBottom.getVelocity(), desiredState.shot.bottomVelocity, FLYWHEEL_TOLERANCE)
                && TorqueMath.toleranced(rotary.getPosition(), desiredState.shot.angle, ROTARY_TOLERANCE);
    }

    public boolean isRotaryAtState() {
        return TorqueMath.toleranced(rotary.getPosition(), desiredState.shot.angle, ROTARY_TOLERANCE);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        if (intake.isIntaking()) {
            desiredState = State.INTAKE;
            gateState = GateState.IN;
        } else if (intake.isCurrentSpike()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }

        final Shot shot = desiredState == State.SMART ? shotTable.get(perception.getDistanceToSpeaker())
                : desiredState.shot;

        if (isReadyToShoot() && desiredState.allowedToShoot) {
            gateState = GateState.OUT;
            Input.getInstance().setRumbleFor(.2);
        }

        flywheelTop.setVolts(flywheelPID.calculate(flywheelTopEncoder.getVelocity().getValue(), shot.topVelocity)
                + flywheelFF.calculate(shot.topVelocity));

        flywheelBottom.setVolts(flywheelPID.calculate(flywheelBottomEncoder.getVelocity().getValue(), shot.bottomVelocity)
                        + flywheelFF.calculate(shot.bottomVelocity));
        rotary.setVolts(rotaryPID.calculate(rotaryEncoder.getAbsolutePosition().getValue(), shot.angle));

        gate.setVolts(gateState.voltage);

        if (mode.isTeleop()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }
    }

    public static synchronized final Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }
}