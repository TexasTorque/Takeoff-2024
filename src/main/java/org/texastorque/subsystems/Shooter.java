package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueLookUpTable;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends TorqueStatorSubsystem<Shooter.State> implements Subsystems {

    private static volatile Shooter instance;

    public static record Shot(double topVelocity, double bottomVelocity, double angle) {
        public Shot(double velocity, double angle) {
            this(velocity, velocity, angle);
        }

        private static final Shot empty = new Shot(0, 0, 0);
    }

    private static final double ROTARY_OFF_POSITION = 115;

    public static enum State implements TorqueState {
        OFF(new Shot(0, ROTARY_OFF_POSITION), false),
        AUTO_OFF(new Shot(0, 90), false),
        SMART_WARMUP(new Shot(5500, 25), false),
        INTAKE(new Shot(-1900, 194), false),
        FAST_INTAKE(new Shot(-2000, 194), false),
        BABYBIRD(new Shot(-1200, 90), false),
        AMP(new Shot(1000, 62), true),
        TRAP(new Shot(0, 108), true),
        LAYUP(new Shot(3900, 63), true),
        MID(new Shot(4300, 37), true),
        SAFEZONE(new Shot(4300, 33), true),
        SMART(true);

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
        OFF(0), IN(8), OUT(-8);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    private static final double FLYWHEEL_TOLERANCE = 120, ROTARY_TOLERANCE = 3;

    private final TorqueNEO rotary, flywheelTop, flywheelBottom, gate;

    private final CANcoder rotaryEncoder, flywheelTopEncoder, flywheelBottomEncoder;

    private final PIDController rotaryPID, flywheelTopPID, flywheelBottomPID;

    private final SimpleMotorFeedforward flywheelFF;

    private final TorqueLookUpTable<Shot> shotTable;

    private final DigitalInput noteSensor;

    private final Timer shootingWarmupTimer = new Timer(), intakeWarmupTimer = new Timer();

    private GateState gateState = GateState.OFF;

    private boolean debugMode = false, idle = true, consent = false;

    private Shot shot = new Shot(0, 0);

    public static double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }

    public void setConsent(boolean consent) {
        this.consent = consent;
    }

    public Shooter() {
        super(State.OFF);

        rotary = new TorqueNEO(Ports.SHOOTER_ROTARY);
        rotary.setVoltageCompensation(12.6);
        rotary.setCurrentLimit(35);
        rotary.setBreakMode(true);
        rotary.invertMotor(true);
        rotary.burnFlash();

        rotaryPID = new PIDController(.15, 0, 0);
        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);

        flywheelTop = new TorqueNEO(Ports.FLYWHEEL_TOP);
        flywheelTop.setCurrentLimit(60);
        flywheelTop.setVoltageCompensation(12.6);
        flywheelTop.setBreakMode(true);
        flywheelTop.invertMotor(false);
        flywheelTop.burnFlash();

        flywheelTopEncoder = new CANcoder(Ports.FLYWHEEL_TOP_ENCODER);
        flywheelTopPID = new PIDController(0.003, 0, 0);

        flywheelBottom = new TorqueNEO(Ports.FLYWHEEL_BOTTOM);
        flywheelBottom.setCurrentLimit(60);
        flywheelBottom.setVoltageCompensation(12.6);
        flywheelBottom.setBreakMode(true);
        flywheelBottom.invertMotor(true);
        flywheelBottom.burnFlash();

        flywheelBottomEncoder = new CANcoder(Ports.FLYWHEEL_BOTTOM_ENCODER);
        flywheelBottomPID = new PIDController(0.003, 0, 0);

        flywheelFF = new SimpleMotorFeedforward(0.000, 0.0011, 0);

        gate = new TorqueNEO(Ports.SHOOTER_GATE);
        gate.setVoltageCompensation(12.6);
        gate.setCurrentLimit(25);
        gate.setBreakMode(true);

        noteSensor = new DigitalInput(Ports.SHOOTER_NOTE_SENSOR);

        shotTable = new TorqueLookUpTable<Shot>(
                (final Shot me, final Shot other) -> Math.abs(other.angle - me.angle) < 1
                        && Math.abs(other.topVelocity - me.topVelocity) < 10,
                (final Shot me, final Shot end, final Double t) -> new Shot(lerp(me.topVelocity, end.topVelocity, t),
                        lerp(me.angle, end.angle, t)));

        shotTable.add(1.22, new Shot(4100, 64));
        // shotTable.add(1.75, new Shot(4200, 50.4));
        // shotTable.add(2.28, new Shot(4500, 43.2));
        // shotTable.add(2.5, new Shot(4600, 41));
        // shotTable.add(2.77, new Shot(4700, 36));
        // shotTable.add(3.39, new Shot(4900, 30.6));
        // shotTable.add(4.1, new Shot(5250, 28.8));
        // shotTable.add(4.7, new Shot(5550, 24.8));
        // shotTable.add(4.9, new Shot(6250, 21));
        // shotTable.add(5.4, new Shot(6250, 18.8));

        shotTable.add(1.85, new Shot(3900, 50));
        shotTable.add(2.84, new Shot(4300, 37));
        shotTable.add(3.15, new Shot(4400, 35));
        shotTable.add(3.3, new Shot(4500, 32));
        shotTable.add(3.5, new Shot(4550, 31));
        shotTable.add(3.6, new Shot(4500, 30));
        shotTable.add(4.1, new Shot(4600, 29));
        shotTable.add(4.8, new Shot(4700, 25.5));
        shotTable.add(5.6, new Shot(4900, 22));

        SmartDashboard.putNumber("Shot Velocity", 0);
        SmartDashboard.putNumber("Shot Angle", 0);

        SmartDashboard.putNumber("Amp Percent", .13);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    public boolean hasNote() {
        return !noteSensor.get();
    }

    private int loopsThatShooterHasBeenReadyToShoot = 0;

    public boolean isReadyToShoot() {
        if (!wantsToShoot())
            return false;
        return isTopFlywheelReady() && isBottomFlywheelReady() && isRotaryAtState();
    }

    public double getRotaryEncoder() {
        return rotaryEncoder.getAbsolutePosition().getValue() * 360;
    }

    private double getTopFlywheelVelocity() {
        return flywheelTopEncoder.getVelocity().getValue() * 60;
    }

    private double getBottomFlywheelVelocity() {
        return flywheelBottomEncoder.getVelocity().getValue() * 60;
    }

    private boolean isTopFlywheelReady() {
        return Math.abs(Math.abs(getTopFlywheelVelocity()) -
                Math.abs(shot.topVelocity)) <= FLYWHEEL_TOLERANCE;
    }

    private boolean isBottomFlywheelReady() {
        return Math.abs(Math.abs(getBottomFlywheelVelocity()) -
                Math.abs(shot.bottomVelocity)) <= FLYWHEEL_TOLERANCE;
    }

    public boolean isRotaryAtState() {
        return Math.abs(getRotaryEncoder() - shot.angle) <= ROTARY_TOLERANCE;
    }

    public boolean wantsToShoot() {
        return desiredState.isAShot;
    }

    public void setIdle(boolean idle) {
        this.idle = idle;
    }

    public boolean inDebugMode() {
        return debugMode;
    }

    public void setDebugMode(boolean mode) {
        debugMode = mode;
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("Shooter State", desiredState.toString());
        Debug.log("Shooter Rotary Positon", getRotaryEncoder());
        Debug.log("Shooter Top Velocity", getTopFlywheelVelocity());
        Debug.log("Shooter Bottom Velocity", -getBottomFlywheelVelocity());
        Debug.log("Shooter Gate Current", gate.getCurrent());
        Debug.log("Shooter is Ready", isReadyToShoot());
        Debug.log("Distance to Tag", perception.getDistanceToSpeaker());
        Debug.log("Angle to Speaker", perception.getAngleToSpeaker().getDegrees());
        Debug.log("Angle to Speaker Rembrand", perception.getAngleToSpeakerRembrandt().getDegrees());
        Debug.log("Top Flywheel Ready", isTopFlywheelReady());
        Debug.log("Bottom Flywheel Ready", isBottomFlywheelReady());
        Debug.log("Rotary Ready", isRotaryAtState());
        Debug.log("Has Note", hasNote());
        Debug.log("Debug Mode", debugMode);

        Debug.log("Flywheel Top Current", flywheelTop.getCurrent());
        Debug.log("Flywheel Bottom Current", flywheelBottom.getCurrent());

        Debug.log("Flywheel Top Voltage", flywheelTop.getVolts());
        Debug.log("Flywheel Bottom Voltage", flywheelBottom.getVolts());

        Debug.log("drivebase aligned", drivebase.isAligned());
        Debug.log("Idle Mode", idle);
        Debug.log("Shooter Shot Velocity", shot.topVelocity);

        if (mode.isTeleop() && intake.isIntaking()) {
            if (intake.isAtState() && !hasNote()) {
                desiredState = State.INTAKE;
                gateState = GateState.IN;
            } else if (hasNote()) {
                desiredState = State.OFF;
                gateState = GateState.OFF;
            }
        }

        if (intake.isIntaking()) {
            flywheelBottom.setCurrentLimit(80);
            flywheelTop.setCurrentLimit(80);
        } else {
            flywheelBottom.setCurrentLimit(60);
            flywheelTop.setCurrentLimit(60);
        }

        if (desiredState == State.INTAKE && mode.isAuto()) {
            desiredState = State.FAST_INTAKE;
        }

        if (wantsState(State.SMART)) {
            shot = shotTable.get(perception.getDistanceToSpeaker() + (mode.isAuto() ? 0 : 0));
            drivebase.setAlignTarget(perception.getAngleToSpeakerRembrandt());
        } else {
            shot = desiredState.shot;
        }

        // Testing to get new data points
        if (wantsState(State.SMART) && debugMode && mode.isTeleop()) {
            double velo = SmartDashboard.getNumber("Shot Velocity", -1);
            double angle = TorqueMath.constrain(SmartDashboard.getNumber("Shot Angle", -1), 0, 190);
            shot = new Shot(velo, angle);
        }

        if (isReadyToShoot())
            loopsThatShooterHasBeenReadyToShoot++;
        else if (!wantsToShoot())
            loopsThatShooterHasBeenReadyToShoot = 0;

        if (loopsThatShooterHasBeenReadyToShoot > 15) // && consent (if mode.isTeleop())
            gateState = GateState.OUT;

        Debug.log("Shot", shot.toString());

        if (wantsState(Shooter.State.OFF) && hasNote() && mode.isTeleop() && idle) {
            final double desiredVolts = TorqueMath.constrain(Math.pow(shootingWarmupTimer.get(), 2) * .25, 0, 3.5);

            flywheelTop.setVolts(desiredVolts);
            flywheelBottom.setVolts(desiredVolts);

        } else if (wantsState(Shooter.State.OFF) && mode.isTeleop() && idle) {
            final double desiredVolts = -TorqueMath.constrain(Math.pow(intakeWarmupTimer.get(), 2) * .25, 0, 2.5);
            flywheelTop.setVolts(desiredVolts);
            flywheelBottom.setVolts(desiredVolts);
        } else if (wantsState(State.AMP)) {
            // double ampPercent = SmartDashboard.getNumber("Amp Percent", .1375);
            double ampPercent = .1375;
            flywheelTop.setPercent(ampPercent);
            flywheelBottom.setPercent(ampPercent);
        } else {
            flywheelTop.setVolts(flywheelTopPID.calculate(getTopFlywheelVelocity(), shot.topVelocity)
                    + flywheelFF.calculate(shot.topVelocity));
            flywheelBottom.setVolts(flywheelBottomPID.calculate(-getBottomFlywheelVelocity(), shot.bottomVelocity)
                    + flywheelFF.calculate(shot.bottomVelocity));

            intakeWarmupTimer.restart();
            shootingWarmupTimer.restart();
        }

        rotary.setVolts(TorqueMath.constrain(rotaryPID.calculate(getRotaryEncoder(),
                shot.angle), wantsState(State.AMP) ? 8 : 8));

        Debug.log("Shooter Gate State", gateState.toString());

        gate.setVolts(gateState.voltage);

        if (mode.isTeleop()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }

    }

    @Override
    public void clean(TorqueMode mode) {
    }

    public void setGateState(GateState state) {
        this.gateState = state;
    }

    public TorqueCommand yieldGateState(GateState state) {
        return new TorqueRun(() -> setGateState(state));
    }

    public static synchronized final Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }
}