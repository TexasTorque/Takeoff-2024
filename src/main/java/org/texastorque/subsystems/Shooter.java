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

public class Shooter extends TorqueStatorSubsystem<Shooter.State> implements Subsystems {

    private static volatile Shooter instance;

    public static record Shot(double velo, double angle) { 
        private static final Shot empty = new Shot(0, 0);
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
        public final boolean allowed2shoot;

        private State(final boolean allowed2shoot) {
            this(Shot.empty, allowed2shoot);
        }

        private State(final Shot shot, final boolean allowed2shoot) {
            this.shot = shot;
            this.allowed2shoot = allowed2shoot;
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

    private final TorqueNEO rotary, flywheels, gate;

    private final CANcoder rotaryEncoder;

    private final PIDController rotaryPID;

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

        flywheels = new TorqueNEO(Ports.FLYWHEEL_LEFT);
        flywheels.addFollower(Ports.FLYWHEEL_RIGHT, true);
        flywheels.setVoltageCompensation(12.6);
        flywheels.setBreakMode(false);

        gate = new TorqueNEO(Ports.SHOOTER_GATE);
        gate.setVoltageCompensation(12.6);
        gate.setBreakMode(true);

        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);
        rotaryPID = new PIDController(1, 0, 0);

        shotTable = new TorqueLookUpTable<Shot>(
            (final Shot me, final Shot other) -> Math.abs(other.angle - me.angle) < 0.1 && Math.abs(other.velo - me.velo) < 0.1, 
            (final Shot me, final Shot end, final Double t) -> new Shot(lerp(me.velo, end.velo, t), lerp(me.angle, end.angle, t))); 
    }

    public boolean hasGateSpiked() {
        return gate.getCurrent() >= GATE_CURRENT_SPIKE;
    }

    public boolean readyToShoot() {
        return TorqueMath.toleranced(flywheels.getVelocity(), desiredState.shot.velo, FLYWHEEL_TOLERANCE) &&
                TorqueMath.toleranced(rotary.getPosition(), desiredState.shot.angle, ROTARY_TOLERANCE);
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
        }

        if (readyToShoot() && desiredState.allowed2shoot) {
            gateState = GateState.OUT;
            Input.getInstance().setRumbleFor(.2);
        }

        final Shot shot = desiredState == State.SMART
                ? shotTable.get(perception.getDistanceToSpeaker())
                : desiredState.shot;

        flywheels.setVelocity(shot.velo);
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
