/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
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
import org.texastorque.torquelib.control.TorqueRollingMean;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorquePolyRegression;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shooter subsystem. Contains shooter flywheels, rotary, and gate.
 */
public class Shooter extends TorqueStatorSubsystem<Shooter.State> implements Subsystems {
    private static volatile Shooter instance;

    /**
     * Shot record contains information about the shot that the shooter should
     * shoot.
     */
    public static record Shot(double topVelocity, double bottomVelocity, Rotation2d angle) {
        public Shot(double velocity, Rotation2d angle) {
            this(velocity, velocity, angle);
        }

        private static final Shot empty = new Shot(0, 0, new Rotation2d(0));
    }

    private static final Rotation2d ROTARY_OFF_POSITION = Rotation2d.fromDegrees(111);

    /**
     * Shooter states usually contain a shot. Sometimes they contain an empty shot
     * b/c the
     * shot is overridden later with other code. We can also specify if the state is
     * a shot
     * such that it can auto shoot.
     */
    public static enum State implements TorqueState {
        OFF(new Shot(0, ROTARY_OFF_POSITION), false),
        AUTO_OFF(new Shot(0, Rotation2d.fromDegrees(99)), false),

        // These shots run the flywheels backwards because they are used for intaking
        // the note
        // into the shooter
        INTAKE(new Shot(-2500, Rotation2d.fromDegrees(198)), false),
        BABYBIRD(new Shot(-2500, Rotation2d.fromDegrees(79)), false),

        // Amp and trap are special setpoints
        AMP(new Shot(1600, Rotation2d.fromDegrees(84)), false), // 80.7 <-- real angle
        AMP_INIITAL(new Shot(0, Rotation2d.fromDegrees(125)), false),
        CLIMB(new Shot(0, Rotation2d.fromDegrees(84)), false),
        TRAP(new Shot(2000, Rotation2d.fromDegrees(81)), true),

        // Layup, black line, and safe zone backup shots. These have reversable "shift
        // shots"
        LAYUP(new Shot(4200, Rotation2d.fromDegrees(64)), new Shot(4200, Rotation2d.fromDegrees(121)), true),

        // Used for tossing a note across the field
        LASER(new Shot(3400, Rotation2d.fromDegrees(54)), true),

        MID(new Shot(4400, Rotation2d.fromDegrees(41)), new Shot(4400, Rotation2d.fromDegrees(135)), true),
        SAFEZONE(new Shot(4600, Rotation2d.fromDegrees(37)), new Shot(4300, Rotation2d.fromDegrees(143)), true),

        // Future and Smart shots are special.
        // - Smart: will be overridden later w/ a calculated shot for our *current*
        // position
        // - Future: will be overridden later w/ a calculated shot for our *future*
        // position
        // this future position is stored in a class variable in the shooter.
        FUTURE_SMART(true),
        SMART_WARMUP(new Shot(5000, Rotation2d.fromDegrees(45)), false),
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

    /**
     * Gate state is a specialty state used for the gate motor.
     */
    public static enum GateState implements TorqueState {
        OFF(0), IN(8), OUT(-8);

        private final double voltage;

        private GateState(final double voltage) {
            this.voltage = voltage;
        }
    }

    /**
     * Chute state is a specialty state used for the flap motor.
     */
    public static enum ChuteState implements TorqueState {
        IN(Rotation2d.fromDegrees(170)), OUT(Rotation2d.fromDegrees(331.2));

        private Rotation2d position;

        private ChuteState(final Rotation2d position) {
            this.position = position;
        }
    }

    private static final double FLYWHEEL_TOLERANCE = 200, ROTARY_TOLERANCE = 1.5, MAX_SHOT_VELO_RPM = 5500,
            FLYWHEEL_ERROR_TOLERANCE = 120, ROTARY_ERROR_TOLERANCE = .5, CHUTE_TOLERANCE = 20, CHUTE_OFFSET = 346;

    // Subsystem hardware...
    private final TorqueNEO rotary, flywheelTop, flywheelBottom, gate, chute;
    private final CANcoder rotaryEncoder, flywheelTopEncoder, flywheelBottomEncoder, chuteEncoder;
    private final DigitalInput noteSensor;

    // ...and controllers
    private final PIDController rotaryPID, flywheelTopPID, flywheelBottomPID, chutePID;
    private ArmFeedforward rotaryFF, chuteFF;
    private final SimpleMotorFeedforward flywheelFF;

    /**
     * We use a polynomial regression for each parameter of our shot: rpm and angle.
     * 
     * RPM regression is 1st order. y ~ x
     * Angle regression is inv. second order y ~ x^-2
     */
    private final TorquePolyRegression rpmRegression, angleRegression;

    /**
     * Timers used for various functionality:
     * - Shooting warmup: reving up the shooter for shooting when we have a note.
     * - Intaking warmup: reving up the shooter for intaking when we dont have a
     * note.
     */
    private final Timer shootingWarmupTimer = new Timer(),
            intakeWarmupTimer = new Timer();

    /**
     * The global gate and chute state and current shot.
     */
    private GateState gateState = GateState.OFF;
    private ChuteState chuteState = ChuteState.IN;
    private Shot shot = new Shot(0, Rotation2d.fromDegrees(0));

    /**
     * This tracks how many program iterations have been completed where the shooter
     * has been ready to shoot. We wait until we have some set iterations where we
     * are in a good state to shoot so we avoid shooting too quickly based on noise.
     */
    private int loopsThatShooterHasBeenReadyToShoot = 0;

    private TorqueRollingMean flywheelBottomMean, flywheelTopMean, rotaryMean;

    /**
     * These are special cases that the subsystem needs to be aware of.
     * - debugMode: used to turn on shooting with values set over SmartDashboard.
     * - idle: if the robot is allowed to rev up for shooting or intaking.
     * - consent: if the robot is allowed to automatically shoot right now.
     * - emergencyCurrentLimit: if the robot is allowed to draw maximum current
     * on the flywheels (80A).
     * - shift: if the subsystem is requested to try and shoot the shot's shifted
     * version (out the front of the robot).
     * 
     * Protip to future developers: do not use this many boolean edge case states,
     * its not a great practice.
     */
    private boolean debugMode = false,
            idle = true,
            consent = false,
            emergencyCurrentLimit = false,
            shift = false;

    // Setters for above variables...
    public void setIdle(boolean idle) {
        this.idle = idle;
    }

    public void setConsent(boolean consent) {
        this.consent = consent;
    }

    public void setEmergencyCurrentLimit(final boolean limit) {
        emergencyCurrentLimit = limit;
    }

    public void setShift(boolean shift) {
        this.shift = shift;
    }

    public void setDebugMode(boolean mode) {
        debugMode = mode;
    }

    public Shooter() {
        super(State.OFF);

        // *** CONFIGURE ROTARY MOTOR AND CONTROL LOOPS ***

        rotary = new TorqueNEO(Ports.SHOOTER_ROTARY);
        rotary.setVoltageCompensation(12.6);
        rotary.setCurrentLimit(35);
        rotary.setBreakMode(true);
        rotary.invertMotor(true);
        rotary.burnFlash();

        rotaryPID = new PIDController(.25, 0, 0);
        rotaryFF = new ArmFeedforward(0, .57, 0);
        rotaryEncoder = new CANcoder(Ports.SHOOTER_ROTARY_ENCODER);

        // *** CONFIGURE TOP ROTARY MOTOR AND CONTROL LOOPS ***

        flywheelTop = new TorqueNEO(Ports.FLYWHEEL_TOP);
        flywheelTop.setCurrentLimit(40);
        flywheelTop.setVoltageCompensation(12.6);
        flywheelTop.setBreakMode(true);
        flywheelTop.invertMotor(false);
        flywheelTop.burnFlash();

        flywheelTopEncoder = new CANcoder(Ports.FLYWHEEL_TOP_ENCODER);
        flywheelTopPID = new PIDController(0.003, 0, 0);

        // *** CONFIGURE BOTTOM ROTARY MOTOR AND CONTROL LOOPS ***

        flywheelBottom = new TorqueNEO(Ports.FLYWHEEL_BOTTOM);
        flywheelBottom.setCurrentLimit(40);
        flywheelBottom.setVoltageCompensation(12.6);
        flywheelBottom.setBreakMode(true);
        flywheelBottom.invertMotor(true);
        flywheelBottom.burnFlash();

        flywheelBottomEncoder = new CANcoder(Ports.FLYWHEEL_BOTTOM_ENCODER);
        flywheelBottomPID = new PIDController(0.003, 0, 0);

        // *** CONFIGURE FLYWHEEL FEED FORWARD ***

        flywheelFF = new SimpleMotorFeedforward(0.000, 0.0011, 0);

        // *** CONFIGURE GATE MOTOR ***

        gate = new TorqueNEO(Ports.SHOOTER_GATE);
        gate.setVoltageCompensation(12.6);
        gate.setCurrentLimit(25);
        gate.setBreakMode(true);

        // *** CONFIGURE CHUTE MOTOR AND CONTROL LOOPS ***

        chute = new TorqueNEO(Ports.CHUTE);
        chute.setVoltageCompensation(12.6);
        chute.setCurrentLimit(25);
        chute.setBreakMode(true);

        chuteEncoder = new CANcoder(Ports.CHUTE_ENCODER);
        chutePID = new PIDController(.05, 0, 0);
        chuteFF = new ArmFeedforward(0, .4, 0);

        // *** CONFIGURE NOTE SENSOR ***

        noteSensor = new DigitalInput(Ports.SHOOTER_NOTE_SENSOR);

        // *** CONFIGURE SHOOTER REGRESSION DATAPOINTS ***

        final TreeMap<Double, Shot> shotTable = new TreeMap<Double, Shot>();

        shotTable.put(1.19, new Shot(4200, Rotation2d.fromDegrees(64)));
        shotTable.put(1.61, new Shot(4400, Rotation2d.fromDegrees(56)));
        shotTable.put(2.2, new Shot(4600, Rotation2d.fromDegrees(47)));
        shotTable.put(2.63, new Shot(4800, Rotation2d.fromDegrees(41)));
        shotTable.put(3.08, new Shot(5000, Rotation2d.fromDegrees(36)));
        shotTable.put(3.58, new Shot(5200, Rotation2d.fromDegrees(33)));
        shotTable.put(4., new Shot(5400, Rotation2d.fromDegrees(31)));
        shotTable.put(4.5, new Shot(5600, Rotation2d.fromDegrees(29.5)));
        shotTable.put(4.9, new Shot(5800, Rotation2d.fromDegrees(29)));

        Set<Entry<Double, Shot>> entries = shotTable.entrySet();
        double[] distances = new double[entries.size()];
        double[] rpms = new double[entries.size()];
        double[] angles = new double[entries.size()];

        int i = 0;
        for (final Map.Entry<Double, Shot> entry : shotTable.entrySet()) {
            distances[i] = entry.getKey();
            rpms[i] = entry.getValue().bottomVelocity;
            angles[i] = entry.getValue().angle.getDegrees();
            i++;
        }

        rpmRegression = new TorquePolyRegression(distances, rpms, 1);
        angleRegression = new TorquePolyRegression(distances, angles, 2);

        flywheelBottomMean = new TorqueRollingMean(5);
        flywheelTopMean = new TorqueRollingMean(5);
        rotaryMean = new TorqueRollingMean(5);

        // *** SMARTDASHBOARD ENTRIES FOR DEBUG MODE ***

        SmartDashboard.putNumber("Shot Velocity", 0);
        SmartDashboard.putNumber("Shot Angle", 0);
        SmartDashboard.putNumber("Rotary Max Volts", 8);

        SmartDashboard.putNumber("Rotary PID P", 0);
        SmartDashboard.putNumber("Rotary PID I", 0);
        SmartDashboard.putNumber("Rotary PID D", 0);

        SmartDashboard.putNumber("Rotary FF kS", 0);
        SmartDashboard.putNumber("Rotary FF kG", 0);
        SmartDashboard.putNumber("Rotary FF kV", 0);

        SmartDashboard.putNumber("CHUTE PID P", 0);
        SmartDashboard.putNumber("CHUTE PID I", 0);
        SmartDashboard.putNumber("CHUTE PID D", 0);

        SmartDashboard.putNumber("Chute FF G", 0);

        SmartDashboard.putNumber("Chute Amp Position", 0);
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    /** Does the shooter have a note inside? */
    public boolean hasNote() {
        return !noteSensor.get();
    }

    /** Is the shooter trying to do a shift shot? */
    public boolean isShift() {
        return shift;
    }

    /** Is the shooter ready to shoot or not? */
    public boolean isReadyToShoot() {
        if (!wantsToShoot()) {
            return false;
        }
        return isTopFlywheelReady() && isBottomFlywheelReady() && isRotaryAtState();
    }

    public boolean hasBeenReadyToShoot() {
        return loopsThatShooterHasBeenReadyToShoot > loopsOK();

        // double flywheelBottomMeanError = flywheelBottomMean
        // .calculate(Math.abs(shot.bottomVelocity) -
        // Math.abs(getBottomFlywheelVelocity()));
        // double flywheelTopMeanError = flywheelTopMean
        // .calculate(Math.abs(shot.topVelocity) - Math.abs(getTopFlywheelVelocity()));
        // double rotaryMeanError = rotaryMean
        // .calculate(Math.abs(shot.angle.getDegrees()) -
        // Math.abs(getRotaryEncoderDegrees()));

        // return wantsToShoot() && flywheelBottomMeanError <= FLYWHEEL_ERROR_TOLERANCE
        // && flywheelTopMeanError <= FLYWHEEL_ERROR_TOLERANCE
        // && rotaryMeanError <= ROTARY_ERROR_TOLERANCE;
    }

    /** Get the degree angle measured by the rotary encoder. */
    public double getRotaryEncoderDegrees() {
        double rawValueDegrees = rotaryEncoder.getAbsolutePosition().getValue() * 360;
        return rawValueDegrees > 200 ? 9 : rawValueDegrees + 9;
    }

    /** Get the RPM velocity measured by the top flywheel encoder. */
    private double getTopFlywheelVelocity() {
        return flywheelTopEncoder.getVelocity().getValue() * 60;
    }

    /** Get the RPM velocity measured by the bottom flywheel encoder. */
    private double getBottomFlywheelVelocity() {
        return flywheelBottomEncoder.getVelocity().getValue() * 60;
    }

    /** Is the top flywheel ready to shoot the current shot? */
    private boolean isTopFlywheelReady() {
        return Math.abs(Math.abs(getTopFlywheelVelocity()) - Math.abs(shot.topVelocity)) <= FLYWHEEL_TOLERANCE;
    }

    /** Is the top flywheel ready to shoot the current shot? */
    private boolean isBottomFlywheelReady() {
        return Math.abs(Math.abs(getBottomFlywheelVelocity()) - Math.abs(shot.bottomVelocity)) <= FLYWHEEL_TOLERANCE;
    }

    /** Is the rotary at the current shot angle? */
    public boolean isRotaryAtState() {
        return Math.abs(getRotaryEncoderDegrees() - shot.angle.getDegrees()) <= ROTARY_TOLERANCE;
    }

    /**
     * Misc. state observers
     */
    public boolean wantsToShoot() {
        return desiredState.isAShot;
    }

    public boolean inDebugMode() {
        return debugMode;
    }

    public boolean hasConsent() {
        return consent;
    }

    /**
     * Get the current regression shot at some given distance.
     * 
     * @param distance Distance away from goal in meters.
     * @return The shot for that corresponding distance.
     */
    public Shot getRegressionShot(final double distance) {
        // Constrain the RPM on regression shots to be under the max shot velocity.
        final double rpm = TorqueMath.constrain(rpmRegression.predict(distance), 0, MAX_SHOT_VELO_RPM);
        // Constrain the angle on regression shots to be between 0 and 90 degrees.
        final double angle = TorqueMath.constrain(angleRegression.predict(distance), 0, 90);

        return new Shot(rpm, Rotation2d.fromDegrees(angle));
    }

    /**
     * Set the flywheel current limits during the update loop w/o causing
     * CAN overutalization.
     */
    private void setFlywheelCurrentLimits(final int limit) {
        flywheelBottom.setCurrentLimitUpdatable(limit);
        flywheelTop.setCurrentLimitUpdatable(limit);
    }

    /**
     * If the shooter wants to climb
     */
    public boolean wantsToClimb() {
        return wantsState(State.CLIMB) || wantsState(State.TRAP);
    }

    public boolean isChuteReadyForAmp() {
        return chuteState == ChuteState.OUT && ((getChuteEncoderDegrees() >= ChuteState.OUT.position.getDegrees()) || TorqueMath
                .toleranced(getChuteEncoderDegrees(), chuteState.position.getDegrees(), CHUTE_TOLERANCE));
    }

    public double getChuteEncoderDegrees() {
        if (chuteEncoder.getAbsolutePosition().getValue() <= .3)
            return 360;
        else
            return chuteEncoder.getAbsolutePosition().getValue() * 360;
    }

    public double getChuteEncoderActualDegrees() { // relative to the rotary
        return getRotaryEncoderDegrees() + (getChuteEncoderDegrees() - CHUTE_OFFSET);
    }

    public double getChuteSetpointActualRadians(double position) {
        return Rotation2d.fromDegrees(getRotaryEncoderDegrees() + (position - CHUTE_OFFSET)).getRadians();
    }

    @Override
    public void update(TorqueMode mode) {

        // *** SMARTDASHBOARD ENTRIES FOR DEBUG PURPOSES ***
        Debug.log("Shooter State", desiredState.toString());
        Debug.log("Shooter Rotary Positon", getRotaryEncoderDegrees());
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
        Debug.log("Shooter Shot Angle", shot.angle.getDegrees());
        Debug.log("Shooter Consent", consent);
        Debug.log("Shooter Shift", shift);
        Debug.log("Regression Shot", getRegressionShot(perception.getDistanceToSpeaker()).toString());
        Debug.log("Shooter Rotary Positon", getRotaryEncoderDegrees());
        Debug.log("Chute Position Degrees (NO OFFSET)", getChuteEncoderDegrees());
        Debug.log("Chute Encoder Actual Degrees (OFFSET AND ROTARY)", getChuteEncoderActualDegrees());
        Debug.log("Chute Position Actual Radians", getChuteSetpointActualRadians(chuteState.position.getDegrees()));
        Debug.log("Has Been Ready", hasBeenReadyToShoot());

        // rotaryPID.setP(SmartDashboard.getNumber("Rotary PID P", 0));
        // rotaryPID.setI(SmartDashboard.getNumber("Rotary PID I", 0));
        // rotaryPID.setD(SmartDashboard.getNumber("Rotary PID D", 0));

        // double kS = SmartDashboard.getNumber("Rotary FF kS", 0);
        // double kV = SmartDashboard.getNumber("Rotary FF kV", 0);
        // double kG = SmartDashboard.getNumber("Rotary FF kG", 0);

        // rotaryFF = new ArmFeedforward(kS, kG, kV);

        // chutePID.setP(SmartDashboard.getNumber("CHUTE PID P", 0));
        // chutePID.setI(SmartDashboard.getNumber("CHUTE PID I", 0));
        // chutePID.setD(SmartDashboard.getNumber("CHUTE PID D", 0));

        // chuteFF = new ArmFeedforward(0, SmartDashboard.getNumber("Chute FF G", 0),
        // 0);

        // Handle intaking. If we are in teleop and the intake says that we are
        // intaking, then
        // we want to enter our intake condition.
        if (mode.isTeleop() && intake.isIntaking()) {
            // If the intake is down all the way and out of the shooters path, then we can
            // set
            // our own state to intake mode, and make sure that the gate is running inwards.
            if (intake.isAtState() && !hasNote()) {
                desiredState = State.INTAKE;
                gateState = GateState.IN;
                // Otherwise (we do have a note), we want to return back to our stow position
                // because we know we have successfully intaked the note.
            } else if (hasNote()) {
                desiredState = State.OFF;
                gateState = GateState.OFF;
            }
        }

        Debug.log("is Chute Ready", isChuteReadyForAmp());

        if (wantsState(State.AMP) && !isChuteReadyForAmp())
            desiredState = State.AMP_INIITAL;

        Debug.log("Shooter State after Amp", desiredState.toString());

        // Current limit handling.
        if (emergencyCurrentLimit || wantsState(State.AMP)) {
            setFlywheelCurrentLimits(90);
        } else if (mode.isAuto()) {
            setFlywheelCurrentLimits(70);
        } else if (intake.isIntaking()) {
            setFlywheelCurrentLimits(60);
        } else {
            setFlywheelCurrentLimits(40);
        }

        // Set shot parameter and handle shot overridding.
        if (wantsState(State.SMART)) {
            shot = getRegressionShot(perception.getDistanceToSpeaker());
        } else if (wantsState(State.FUTURE_SMART)) {
            shot = getRegressionShot(perception.getFutureDistanceToSpeaker());
        } else {
            shot = desiredState.getShot(shift);
        }

        // Handle debug mode: read shot parameter from smart dashboard.
        if (wantsState(State.SMART) && debugMode && mode.isTeleop()) {
            final double velocity = SmartDashboard.getNumber("Shot Velocity", -1);
            final double angle = TorqueMath.constrain(SmartDashboard.getNumber("Shot Angle", -1), 0, 190);
            shot = new Shot(velocity, Rotation2d.fromDegrees(angle));
        }

        // Handle shooter readiness counter. Increment every loop the shooter is
        // ready to shoot. Set back to zero if the shooter is not ready to shoot.
        // This makes the variable hold the ammount of consecutive iterations that
        // the shooter has been ready to shoot.
        if (isReadyToShoot()) {
            loopsThatShooterHasBeenReadyToShoot++;
        } else if (!wantsToShoot()) {
            loopsThatShooterHasBeenReadyToShoot = 0;
        }

        // Handle auto shooting if the shooter has been ready for some ammount of time.
        if (hasBeenReadyToShoot()) {
            if (mode.isTeleop()) {
                // The drivebase aligned conditional is used for smart shots. If we are in smart
                // shot then we need to check for drivebase being aligned. But, if we are in
                // smart
                // shot AND we are in shift mode, then we should not check for drivebase being
                // aligned.
                final boolean aligned = (wantsState(State.SMART) && !shift) ? drivebase.hasBeenAligned() : true;
                if (consent && aligned) {
                    gateState = GateState.OUT;
                }
            } else if (mode.isAuto()) {
                // If we are in auto then we just need to check that we are not in pathing mode.

                // This is for FUTURE_SHOT:
                // if (!drivebase.wantsState(Drivebase.State.PATHING)) {
                // gateState = GateState.OUT;
                // }

                final boolean aligned = drivebase.hasBeenAligned()
                        || !drivebase.wantsState(Drivebase.State.ALIGN_TO_ANGLE);
                if (!drivebase.wantsState(Drivebase.State.PATHING) && aligned) {
                    gateState = GateState.OUT;
                }
            }
        }

        Debug.log("Shot", shot.toString());
        Debug.log("Gate State", gateState.toString());

        // Handling setting shooter flywheel voltage. Handles idle condition
        // by checking if we are in teleop and we are off and we are good to idle.
        double flywheelVoltsTop = 0, flywheelVoltsBottom = 0;
        if (mode.isTeleop() && wantsState(Shooter.State.OFF) && idle) {
            // We are starting the flywheels for shooter.
            if (hasNote()) {
                flywheelVoltsTop = TorqueMath.constrain(Math.pow(shootingWarmupTimer.get(), 2) * .25, 0, 3.5);
                flywheelVoltsBottom = flywheelVoltsTop;
            }
            // We are starting the flywheels for intake.
            else {
                flywheelVoltsTop = -TorqueMath.constrain(Math.pow(intakeWarmupTimer.get(), 2) * .25, 0, 2.5);
                flywheelVoltsBottom = flywheelVoltsTop;
            }
        } else {
            flywheelVoltsTop = flywheelTopPID.calculate(getTopFlywheelVelocity(), shot.topVelocity)
                    + flywheelFF.calculate(shot.topVelocity);
            flywheelVoltsBottom = flywheelBottomPID.calculate(-getBottomFlywheelVelocity(), shot.bottomVelocity)
                    + flywheelFF.calculate(shot.bottomVelocity);

            intakeWarmupTimer.restart();
            shootingWarmupTimer.restart();
        }

        flywheelTop.setVolts(flywheelVoltsTop);
        flywheelBottom.setVolts(flywheelVoltsBottom);

        Debug.log("State before Amp", desiredState.toString());

        if (wantsState(State.AMP))
            chuteState = ChuteState.OUT;
        else if (wantsState(State.AMP_INIITAL)
                && TorqueMath.toleranced(getRotaryEncoderDegrees(), State.AMP_INIITAL.shot.angle.getDegrees(), 5)) {
            chuteState = ChuteState.OUT;
        } else if (desiredState.isAShot && !shift)
            chuteState = ChuteState.OUT;
        else
            chuteState = ChuteState.IN;

        Debug.log("Shooter Chute State", chuteState.toString());
        Debug.log("Chute Setpoint Position", chuteState.position.getDegrees());

        chute.setVolts(
                -TorqueMath.constrain(chutePID.calculate(getChuteEncoderDegrees(), chuteState.position.getDegrees())
                        + chuteFF.calculate(getChuteSetpointActualRadians(chuteState.position.getDegrees()), 0), 6));

        Debug.log("Chute Volts", -TorqueMath.constrain(
                chutePID.calculate(getChuteEncoderDegrees(), chuteState.position.getDegrees())
                        + chuteFF.calculate(getChuteSetpointActualRadians(chuteState.position.getDegrees()), 0),
                4));

        double rotaryVolts = rotaryPID.calculate(getRotaryEncoderDegrees(), shot.angle.getDegrees())
                + rotaryFF.calculate(shot.angle.getRadians(), 0);

        rotaryVolts = TorqueMath.constrain(rotaryVolts, getRotaryMaxVolts());

        Debug.log("Rotary Volts", rotaryVolts);

        rotary.setVolts(rotaryVolts);

        Debug.log("Shooter Gate State", gateState.toString());

        // Output the gate state voltage.
        gate.setVolts(gateState.voltage);

        if (mode.isTeleop()) {
            desiredState = State.OFF;
            gateState = GateState.OFF;
        }
    }

    /**
     * The ammount of consecutive loops required for the robot to be
     * considered OK to shoot.
     */
    public static int loopsOK() {
        // return DriverStation.isAutonomous() ? 1$a5 : 5;
        return 5;
    }

    /** Compute the maximum allowed voltage for the rotary motor. */
    private double getRotaryMaxVolts() {
        if (wantsState(State.AMP))
            return 3;
        if (wantsState(State.INTAKE) && getRotaryEncoderDegrees() > 140)
            return 3;
        if (debugMode)
            return SmartDashboard.getNumber("Rotary Max Volts", 8);
        return 8;
    }

    @Override
    public void clean(TorqueMode mode) {
    }

    /** Some setters for the gate state */
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