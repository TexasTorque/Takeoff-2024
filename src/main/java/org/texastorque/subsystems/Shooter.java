package org.texastorque.subsystems;

import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.Map.Entry;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorquePolyRegression;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

    private static final double ROTARY_OFF_POSITION = 102;

    public static enum State implements TorqueState {
        OFF(new Shot(0, ROTARY_OFF_POSITION), false),
        TRAP(new Shot(2400, 70), false),
        CLIMB(new Shot(0, 90), false),
        AUTO_OFF(new Shot(0, 90), false),
        INTAKE(new Shot(-2500, 181), false),
        BABYBIRD(new Shot(-1200, 90), false),
        AMP(new Shot(1500, 55), true),
        LAYUP(new Shot(3900, 63), new Shot(3900, 122), true),
        MID(new Shot(4300, 37), new Shot(4300, 133), true),
        SAFEZONE(new Shot(4300, 33), new Shot(4300, 139), true),
        LASER(new Shot(5000, 0), true),
        FUTURE_SMART(true),
        SMART(true);

        public final Shot shot, shiftedShot;
        public final boolean isAShot;

        private State(final boolean isAShot) {
            this(Shot.empty, isAShot);
        }

        private State(final Shot shot, final boolean isAShot) {
            this.shot = shot;
            this.shiftedShot = shot;
            this.isAShot = isAShot;
        }

        private State(final Shot shot, final Shot shiftedShot, final boolean isAShot) {
            this.shot = shot;
            this.shiftedShot = shiftedShot;
            this.isAShot = isAShot;
        }

        public Shot getShot(boolean shift) {
            return shift ? shiftedShot : shot;
        }
    }

    public static enum GateState implements TorqueState {
        OFF(0), IN(8), OUT(-8);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    private static final double FLYWHEEL_TOLERANCE = 120, ROTARY_TOLERANCE = 1.5;

    private final TorqueNEO rotary, flywheelTop, flywheelBottom, gate;

    private final CANcoder rotaryEncoder, flywheelTopEncoder, flywheelBottomEncoder;

    private final PIDController rotaryPID, flywheelTopPID, flywheelBottomPID;

    private final TreeMap<Double, Shot> shotTable;

    private final TorquePolyRegression rpmRegression, angleRegression;

    private final SimpleMotorFeedforward flywheelFF;

    private final DigitalInput noteSensor;

    private final Timer shootingWarmupTimer = new Timer(), intakeWarmupTimer = new Timer();

    private int loopsThatShooterHasBeenReadyToShoot = 0;

    private GateState gateState = GateState.OFF;

    private boolean debugMode = false, idle = true, consent = false, emergencyCurrentLimit = false, shift = false;

    private Shot shot = new Shot(0, 0);

    private double angleOffset = 0;

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

        rotaryPID = new PIDController(.3, 0, 0);
        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);

        flywheelTop = new TorqueNEO(Ports.FLYWHEEL_TOP);
        flywheelTop.setCurrentLimit(40);
        flywheelTop.setVoltageCompensation(12.6);
        flywheelTop.setBreakMode(true);
        flywheelTop.invertMotor(false);
        flywheelTop.burnFlash();

        flywheelTopEncoder = new CANcoder(Ports.FLYWHEEL_TOP_ENCODER);
        flywheelTopPID = new PIDController(0.003, 0, 0);

        flywheelBottom = new TorqueNEO(Ports.FLYWHEEL_BOTTOM);
        flywheelBottom.setCurrentLimit(40);
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

        shotTable = new TreeMap<Double, Shot>();

        shotTable.put(1.22, new Shot(4100, 64));
        shotTable.put(1.75, new Shot(4200, 50.4));
        shotTable.put(2.28, new Shot(4500, 43.2));
        shotTable.put(2.5, new Shot(4600, 41));
        shotTable.put(2.77, new Shot(4700, 36));
        shotTable.put(3.39, new Shot(4900, 30.6));
        shotTable.put(4.1, new Shot(5000, 28.8));
        shotTable.put(4.7, new Shot(5550, 24.8));
        shotTable.put(4.9, new Shot(6250, 21));
        shotTable.put(5.4, new Shot(6250, 18.8));

        Set<Entry<Double, Shot>> entries = shotTable.entrySet();
        double[] distances = new double[entries.size()];
        double[] rpms = new double[entries.size()];
        double[] angles = new double[entries.size()];

        int i = 0;
        for (final Map.Entry<Double, Shot> entry : shotTable.entrySet()) {
            distances[i] = entry.getKey();
            rpms[i] = entry.getValue().bottomVelocity;
            angles[i] = entry.getValue().angle;
            i++;
        }

        rpmRegression = new TorquePolyRegression(distances, rpms, 1);
        angleRegression = new TorquePolyRegression(distances, angles, 2);

        SmartDashboard.putNumber("Shot Velocity", 0);
        SmartDashboard.putNumber("Shot Angle", 0);
        SmartDashboard.putNumber("Angle Offset", 0);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    public void setEmergencyCurrentLimit(final boolean limit) {
        emergencyCurrentLimit = limit;
    }

    public boolean hasNote() {
        return !noteSensor.get();
    }

    public void setShift(boolean shift) {
        this.shift = shift;
    }

    public boolean isShift() {
        return shift;
    }

    public boolean isReadyToShoot() {
        if (!wantsToShoot() || drivebase.wantsState(Drivebase.State.PATHING))
            return false;
        return isReady();
    }

    public boolean isReady() {
        return isTopFlywheelReady() && isBottomFlywheelReady() && isRotaryAtState();
    }

    public double getRotaryEncoder() {
        double rawValueDegrees = rotaryEncoder.getAbsolutePosition().getValue() * 360;
        if (rawValueDegrees > 200)
            return 0;
        else
            return rawValueDegrees;
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

    public boolean wantsToClimb() {
        return wantsState(State.CLIMB) || wantsState(State.TRAP);
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

    public Shot getRegressionShot(double distance) {
        return new Shot(
                TorqueMath.constrain(rpmRegression.predict(distance), 0, DriverStation.isAutonomous() ? 5000 : 7000),
                TorqueMath.constrain(angleRegression.predict(distance), 0, 90));
    }

    public boolean hasConsent() {
        return consent;
    }

    public boolean isDebugMode() {
        return debugMode;
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("Shooter State", desiredState.toString());
        Debug.log("Shooter Rotary Positon", getRotaryEncoder());
        Debug.log("Shooter Top Velocity", getTopFlywheelVelocity());
        Debug.log("Shooter Bottom Velocity", -getBottomFlywheelVelocity());
        Debug.log("Shooter is Ready", isReadyToShoot());
        Debug.log("Distance to Tag", perception.getDistanceToSpeaker());
        Debug.log("Top Flywheel Ready", isTopFlywheelReady());
        Debug.log("Bottom Flywheel Ready", isBottomFlywheelReady());
        Debug.log("Rotary Ready", isRotaryAtState());
        Debug.log("Has Note", hasNote());
        Debug.log("Debug Mode", debugMode);
        Debug.log("Shooter Shot Velocity", shot.topVelocity);
        Debug.log("Shooter Shot Angle", shot.angle);
        Debug.log("Shooter Consent", consent);
        Debug.log("Shooter Shift", shift);

        angleOffset = SmartDashboard.getNumber("Angle Offset", 0);

        if (mode.isTeleop() && intake.isIntaking()) {
            if (intake.isAtState() && !hasNote()) {
                desiredState = State.INTAKE;
                gateState = GateState.IN;
            } else if (hasNote()) {
                desiredState = State.OFF;
                gateState = GateState.OFF;
            }
        }

        if (emergencyCurrentLimit) {
            flywheelBottom.setCurrentLimit(90);
            flywheelTop.setCurrentLimit(90);
        } else if (mode.isAuto()) {
            flywheelBottom.setCurrentLimit(70);
            flywheelTop.setCurrentLimit(70);
        } else if (intake.isIntaking()) {
            flywheelBottom.setCurrentLimit(60);
            flywheelTop.setCurrentLimit(60);
        } else {
            flywheelBottom.setCurrentLimit(40);
            flywheelTop.setCurrentLimit(40);
        }

        if (wantsState(State.SMART))
            shot = getRegressionShot(perception.getDistanceToSpeaker());
        else if (wantsState(State.FUTURE_SMART))
            shot = getRegressionShot(perception.getFutureDistanceToSpeaker());
        else
            shot = desiredState.getShot(shift);

        // Testing to get new data points
        if (wantsState(State.SMART) && debugMode && mode.isTeleop()) {
            double velocity = SmartDashboard.getNumber("Shot Velocity", -1);
            double angle = TorqueMath.constrain(SmartDashboard.getNumber("Shot Angle", -1), 0, 190);
            shot = new Shot(velocity, angle);
        }

        if (isReadyToShoot())
            loopsThatShooterHasBeenReadyToShoot++;
        else if (!wantsToShoot())
            loopsThatShooterHasBeenReadyToShoot = 0;

        if (loopsThatShooterHasBeenReadyToShoot > 15) {
            if ((mode.isTeleop() && consent
                    && ((!shift && wantsState(State.SMART)) ? drivebase.hasBeenAligned() : true)) || mode.isAuto())
                gateState = GateState.OUT;
        }

        Debug.log("Shot", shot.toString());
        Debug.log("Gate State", gateState.toString());

        shot = new Shot(shot.topVelocity, shot.angle + angleOffset);

        if (wantsState(Shooter.State.OFF) && hasNote() && mode.isTeleop() && idle && !isDebugMode()) {
            final double desiredVolts = TorqueMath.constrain(Math.pow(shootingWarmupTimer.get(), 2) * .25, 0, 3.5);
            flywheelTop.setVolts(desiredVolts);
            flywheelBottom.setVolts(desiredVolts);
        } else if (wantsState(Shooter.State.OFF) && mode.isTeleop() && idle && !isDebugMode()) {
            final double desiredVolts = -TorqueMath.constrain(Math.pow(intakeWarmupTimer.get(), 2) * .25, 0, 2.5);
            flywheelTop.setVolts(desiredVolts);
            flywheelBottom.setVolts(desiredVolts);
        } else {
            flywheelTop.setVolts(flywheelTopPID.calculate(getTopFlywheelVelocity(),
                    shot.topVelocity)
                    + flywheelFF.calculate(shot.topVelocity));
            flywheelBottom.setVolts(flywheelBottomPID.calculate(-getBottomFlywheelVelocity(),
                    shot.bottomVelocity)
                    + flywheelFF.calculate(shot.bottomVelocity));

            intakeWarmupTimer.restart();
            shootingWarmupTimer.restart();
        }

        rotary.setVolts(TorqueMath.constrain(rotaryPID.calculate(getRotaryEncoder(), shot.angle),
                (wantsState(State.AMP) || (isDebugMode() ? wantsState(State.SMART) : false)) ? 3 : 8));

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