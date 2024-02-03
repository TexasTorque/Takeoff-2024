package org.texastorque.subsystems;

import org.texastorque.Debug;
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
        OFF(new Shot(0, .32), false),
        INTAKE(new Shot(-1500, .54), false),
        AMP(new Shot(3000, .17), true),
        TRAP(new Shot(0, .3), false),
        WARMUP(new Shot(0, .3), false),
        LAYUP(new Shot(1500, .145), true),
        SAFEZONE(new Shot(5000, .25), true),
        SMART(true),;

        public final Shot shot;
        public final boolean isAShot;

        private State(final boolean isAShot) {
            this(Shot.empty, isAShot);
        }

        private State(final Shot shot, final boolean isAShot) {
            this.shot = shot;
            this.isAShot = isAShot;
        }
    }

    public static enum GateState implements TorqueState {
        OFF(4), IN(12), OUT(-12);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    private static final double FLYWHEEL_TOLERANCE = 100, ROTARY_TOLERANCE = 5, GATE_CURRENT_SPIKE = 10;

    private final TorqueNEO rotary, flywheelTop, flywheelBottom, gate;

    private final CANcoder rotaryEncoder, flywheelTopEncoder, flywheelBottomEncoder;

    private final PIDController rotaryPID, flywheelTopPID, flywheelBottomPID;
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
        rotary.invertMotor(true);
        rotary.burnFlash();

        rotaryPID = new PIDController(100, 0, 0);
        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);

        flywheelTop = new TorqueNEO(Ports.FLYWHEEL_TOP);
        flywheelTop.setVoltageCompensation(12.6);
        flywheelTop.setBreakMode(true);
        flywheelTop.invertMotor(false);
        flywheelTop.burnFlash();

        flywheelTopEncoder = new CANcoder(Ports.FLYWHEEL_TOP_ENCODER);
        flywheelBottomPID = new PIDController(0.001, 0, 0);
        flywheelTopPID = new PIDController(0.001, 0, 0);

        // 0001 about 200 loo low
        // 00025 about 200 too low
        // 001 about 100 too low
        flywheelBottom = new TorqueNEO(Ports.FLYWHEEL_BOTTOM);
        flywheelBottom.setVoltageCompensation(12.6);
        flywheelBottom.setBreakMode(true);
        flywheelBottom.invertMotor(true);
        flywheelBottom.burnFlash();

        flywheelBottomEncoder = new CANcoder(Ports.FLYWHEEL_BOTTOM_ENCODER);

        flywheelFF = new SimpleMotorFeedforward(0.000, 0.001, 0);

        gate = new TorqueNEO(Ports.SHOOTER_GATE);
        gate.setVoltageCompensation(12.6);
        gate.setBreakMode(true);

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
        return TorqueMath.toleranced(Math.abs(flywheelTopEncoder.getVelocity().getValue() * 60),
                desiredState.shot.topVelocity,
                FLYWHEEL_TOLERANCE)
                && TorqueMath.toleranced(Math.abs(flywheelTopEncoder.getVelocity().getValue() * 60),
                        desiredState.shot.bottomVelocity,
                        FLYWHEEL_TOLERANCE);
        // && TorqueMath.toleranced(rotary.getPosition(), desiredState.shot.angle,
        // ROTARY_TOLERANCE);
    }

    public boolean isRotaryAtState() {
        return TorqueMath.toleranced(rotary.getPosition(), desiredState.shot.angle, ROTARY_TOLERANCE);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    private double getTopFlywheelVelocity() {
        return 0;
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("Shooter State", desiredState.toString());
        Debug.log("Shooter Rotary", rotaryEncoder.getAbsolutePosition().getValue());
        Debug.log("Shooter Top Velocity", flywheelTopEncoder.getVelocity().getValue() * 60);
        Debug.log("Shooter Bottom Velocity", flywheelBottomEncoder.getVelocity().getValue() * 60);
        Debug.log("Shooter Req Velocity", desiredState.shot.topVelocity);
        Debug.log("Shooter Req Top Velocity", desiredState.shot.topVelocity);
        Debug.log("Shooter Req Bottom Velocity", desiredState.shot.bottomVelocity);

        Debug.log("is ready to shoot", isReadyToShoot());

        if (intake.isIntaking() && intake.isRotaryAtState()) {
            desiredState = State.INTAKE;
            gateState = GateState.IN;
        } else if (intake.isCurrentSpike()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }

        final Shot shot = desiredState == State.SMART ? shotTable.get(perception.getDistanceToSpeaker())
                : desiredState.shot;

        if (isReadyToShoot() && desiredState.isAShot) {
            gateState = GateState.OUT;
            // Input.getInstance().setRumbleFor(.2);
        }

        double topPID = flywheelTopPID.calculate(flywheelTopEncoder.getVelocity().getValue() * 60, shot.topVelocity)
                + flywheelFF.calculate(shot.topVelocity);

        double bottomPID = flywheelBottomPID.calculate(-flywheelBottomEncoder.getVelocity().getValue() * 60,
                shot.bottomVelocity) + flywheelFF.calculate(shot.bottomVelocity);

        Debug.log("Flywheel Top PID Volts", topPID);
        Debug.log("Flywheel Bottom PID Volts", bottomPID);

        flywheelTop.setVolts(topPID);
        flywheelBottom.setVolts(bottomPID);

        double pidVolts = rotaryPID.calculate(rotaryEncoder.getAbsolutePosition().getValue(), shot.angle);

        Debug.log("Shooter Top Roller Voltage ", flywheelTop.getVolts());
        Debug.log("Shooter Bottom Roller Voltage ", flywheelBottom.getVolts());

        Debug.log("PID Volts", pidVolts);

        rotary.setVolts(TorqueMath.constrain(pidVolts, 8));

        gate.setVolts(gateState.voltage);

        if (mode.isTeleop()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }
    }

    public static synchronized final Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }

    @Override
    public void clean(TorqueMode mode) {
    }
}