package org.texastorque.subsystems;

import java.util.TreeMap;
import java.util.Map;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueLookUpTable;
import org.texastorque.torquelib.control.TorqueLookUpTable.ShotParameter;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import static java.util.Map.entry;

public class Shooter extends TorqueStatorSubsystem<Shooter.State> implements Subsystems {

    private static volatile Shooter instance;

    public static enum State implements TorqueState {
        OFF(0, 0), INTAKE(20, -20), AMP(30, 10), TRAP(30, 20), SETPOINT, SPEAKER_SMART_SHOT, WARMUP;

        private double rotaryPosition, flywheelSpeed;

        private State() {
        }

        private State(double rotaryPosition, double flywheelSpeed) {
            this.rotaryPosition = 0;
            this.flywheelSpeed = 0;
        }
    }

    public static enum GateState implements TorqueState {
        OFF(0), IN(-12), OUT(12);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    public static final ShotParameter SPEAKER_LAYUP = new ShotParameter(30, 20),
            SPEAKER_SAFE_ZONE = new ShotParameter(30, 20), SPEAKER_WARMUP = new ShotParameter(30, 20),
            AMP_WARMUP = new ShotParameter(30, 20);

    private static final double FLYWHEEL_TOLERANCE = 100, ROTARY_TOLERANCE = 5, GATE_CURRENT_SPIKE = 10;

    private final TorqueNEO rotary, flywheels, gate;

    private final CANcoder rotaryEncoder;

    private final PIDController rotaryPID;

    private final TorqueLookUpTable lookUpTable;

    private GateState gateState = GateState.OFF;

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

        TreeMap<Double, ShotParameter> table = new TreeMap<Double, ShotParameter>(
                Map.ofEntries(
                        entry(2., new ShotParameter(30, 5)),
                        entry(2.5, new ShotParameter(40, 5))));

        lookUpTable = new TorqueLookUpTable(table);
    }

    public boolean gateSpike() {
        return gate.getCurrent() >= GATE_CURRENT_SPIKE;
    }

    public boolean readyToShoot() {
        return TorqueMath.toleranced(flywheels.getVelocity(), desiredState.flywheelSpeed, FLYWHEEL_TOLERANCE) &&
                TorqueMath.toleranced(rotary.getPosition(), desiredState.rotaryPosition, ROTARY_TOLERANCE);
    }

    public boolean rotaryIsAtState() {
        return TorqueMath.toleranced(rotary.getPosition(), desiredState.rotaryPosition, ROTARY_TOLERANCE);
    }

    public void setShotParameter(final ShotParameter shot) {
        desiredState.flywheelSpeed = shot.flywheelRPM;
        desiredState.rotaryPosition = shot.hoodAngle;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        if (desiredState == State.SPEAKER_SMART_SHOT)
            setShotParameter(lookUpTable.get(perception.getDistanceToTarget()));

        if ((intake.getState() == Intake.State.SMART_INTAKE || intake.getState() == Intake.State.INTAKE)
                && !intake.rotaryIsAtState())
            desiredState = State.OFF;

        if (readyToShoot()&& desiredState != State.WARMUP) {
            gateState = GateState.OUT;
            Input.getInstance().setRumbleFor(.2);
        }

        if (intake.getState() == Intake.State.SMART_INTAKE
                || intake.getState() == Intake.State.INTAKE && intake.rotaryIsAtState())
            gateState = GateState.IN;

        flywheels.setVelocity(desiredState.flywheelSpeed);

        gate.setVolts(gateState.voltage);

        rotary.setVolts(
                rotaryPID.calculate(rotaryEncoder.getAbsolutePosition().getValue(), desiredState.rotaryPosition));

        if (mode.isTeleop())
            desiredState = State.OFF;
    }

    public static synchronized final Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }
}
