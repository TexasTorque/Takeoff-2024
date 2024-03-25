/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.base.TorqueSwerveModule;
import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwerveConfig;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleKraken;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleNEO;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Swerve drivebase subsystem.
 */
public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State>
        implements Subsystems, TorquePathingDrivebase {

    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN_TO_ANGLE(ROBOT_RELATIVE),
        DASH(FIELD_RELATIVE),
        PATHING(null);
        // ^ PATHING is an extra state to be like ROBOT_RELATIVE but its explicity
        // pathing

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    /**
     * Enum for shifting drivebase speeds, like a transmition. However the benefit
     * is
     * not through increase torque but rather increase controlability.
     */
    public enum SpeedSetting {
        SLOW(.25), MID(.5), FAST(1.0), SEQ(1);

        private static final SpeedSetting[] vals = values();
        public double speed;

        private SpeedSetting(final double speed) {
            this.speed = speed;
        }

        public SpeedSetting shiftUp() {
            return vals[Math.min((this.ordinal() + 1), vals.length - 2)];
        }

        public SpeedSetting shiftDown() {
            return vals[Math.max((this.ordinal() - 1), 0)];
        }
    }

    /**
     * Used to drop the speed setting gradually over time when the speed setting
     * is in the sequence enum state.
     */
    public static class SpeedSequence {
        final double initSpeed, finalSpeed, duration, startTime, speedDeceleration;

        // Linearly decreases the speed every second for a duration of time
        public SpeedSequence(final SpeedSetting initSpeed, final SpeedSetting finalSpeed,
                final double duration) {
            this.initSpeed = initSpeed.speed;
            this.finalSpeed = finalSpeed.speed;
            this.duration = duration;
            speedDeceleration = (this.initSpeed - this.finalSpeed) / duration;
            startTime = Timer.getFPGATimestamp();
        }

        public double get() {
            return Math.max(initSpeed - speedDeceleration * (Timer.getFPGATimestamp() - startTime),
                    finalSpeed);
        }
    }

    private static volatile Drivebase instance;

    /**
     * Returns the normal distance between the rotational axis of the outermost
     * wheel and the center of the robot, used to satisfy the TorquePathingDrivebase
     * interface.
     */
    public double getRadius() {
        return WIDTH * Math.sqrt(2);
    }

    /** Return the maximum translational speed to satisfy TorquePathingDrivebase. */
    public double getMaxSpeed() {
        return MAX_VELOCITY;
    }

    public static final double WIDTH = Units.inchesToMeters(21.25), // distance between swerve axis

            // WARNING: make sure you have the correct one for the modules

            // MAX_VELOCITY = SwerveConfig.swervexNeo.maxVelocity, // maximum translational velocity of the swerve (m/s)
            MAX_VELOCITY = SwerveConfig.swervexKraken.maxVelocity, // maximum translational velocity of the swerve (m/s)

            // WARNING: the above is very important!

            MAX_ACCELERATION = 5, // maximum translation acceleration of the swerver (m/s^2)
            MAX_ANGULAR_VELOCITY = 2 * Math.PI; // maximum rotation velocity of the swerve (rad/s)

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    /**
     * Represents x and y translations between the center of the robot
     * and the rotational axis of each swerve module.
     */
    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    private final TorqueSwerveModule fl, fr, bl, br;

    public TorqueSwerveSpeeds inputSpeeds; // swerve velocity vector
    public final SwerveDriveKinematics kinematics; // used to transform swerve velo vector to module vectors
    private SwerveModuleState[] swerveStates; // used to store the req. swerve module vectors

    // Store speed setting state and an instance of the speed sequence class for
    // gradual slowing.
    public SpeedSetting speedSetting = SpeedSetting.FAST;
    public SpeedSequence speedSequence = new SpeedSequence(speedSetting, speedSetting, -1);

    // Alginment PID controller used for aligning the drivebase to a target angle.
    // The loopsThatDB is aligned field is used to count the number of consecutive
    // update iterations that the drivebase has been aligned to the target angle.
    private final PIDController headingLockPID, offsetTargetingPID;
    private double loopsThatDBIsAligned = 0;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        // For Bravo -- using the swerve-x Neo config.
        // fl = new TorqueSwerveModuleNEO("Front Left", Ports.FL_MOD, SwerveConfig.swervexNeo);
        // fr = new TorqueSwerveModuleNEO("Front Right", Ports.FR_MOD, SwerveConfig.swervexNeo);
        // bl = new TorqueSwerveModuleNEO("Back Left", Ports.BL_MOD, SwerveConfig.swervexNeo);
        // br = new TorqueSwerveModuleNEO("Back Right", Ports.BR_MOD, SwerveConfig.swervexNeo);

        // For Charlie -- using the swerve-x Kraken config.
        fl = new TorqueSwerveModuleKraken("Front Left", Ports.FL_MOD, SwerveConfig.swervexKraken);
        fr = new TorqueSwerveModuleKraken("Front Right", Ports.FR_MOD, SwerveConfig.swervexKraken);
        bl = new TorqueSwerveModuleKraken("Back Left", Ports.BL_MOD, SwerveConfig.swervexKraken);
        br = new TorqueSwerveModuleKraken("Back Right", Ports.BR_MOD, SwerveConfig.swervexKraken);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);
        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);
        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        headingLockPID = new PIDController(.085, 0, 0);
        headingLockPID.enableContinuousInput(0, 360);

        offsetTargetingPID = new PIDController(.032 / 36, 0, 0);

        SmartDashboard.putNumber("Align PID P", 0);
        // maybe make continuous input to something idk?
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        // Set the angle target for ALIGN_TO_ANGLE state, basically makes that state
        // an "align to goal" state.
        setAlignTarget(perception::getHeadingLock);
    }

    public SwerveModulePosition invertSwerveModuleDistance(SwerveModulePosition position) {
        return new SwerveModulePosition(-position.distanceMeters, position.angle);
    }

    /** Get aggregate module positions for feedback. */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                invertSwerveModuleDistance(fl.getPosition()), invertSwerveModuleDistance(fr.getPosition()),
                invertSwerveModuleDistance(bl.getPosition()), invertSwerveModuleDistance(br.getPosition())
        };
    }

    /**
     * An alignment target supplier, used to provide the align target during
     * ALIGN_STATE
     */
    private Supplier<Rotation2d> alignTarget = () -> Rotation2d.fromDegrees(0);

    /** Set alignment target w/ a constant rotation */
    public void setAlignTarget(final Rotation2d target) {
        alignTarget = () -> target;
    }

    /**
     * Set alignment target w/ a constant rotation relative to the current drivebase
     * angle
     */
    public void setAlignTargetRelative(final Rotation2d target) {
        alignTarget = () -> target.plus(perception.getHeading());
    }

    /** Set align target with a supplier. */
    public void setAlignTarget(final Supplier<Rotation2d> target) {
        alignTarget = target;
    }

    /** Grab the constrained align target for PID alignment action */
    private double getAlignTarget() {
        return TorqueMath.constrain0to360(alignTarget.get().getDegrees());
    }

    public static final double ALIGN_TOLERANCE = 2, TARGET_TOLERANCE = 150;

    /**
     * Is the drivebase aligned to the requested angle within an acceptable
     * tolerance
     */
    public boolean isAligned() {
        return TorqueMath.toleranced(perception.getHeading().getDegrees(), getAlignTarget(), ALIGN_TOLERANCE);
    }

    /** Is the target error low enough */
    public boolean isTargetLocked(final double fusedPos) {
        return Math.abs(fusedPos) <= TARGET_TOLERANCE;
    }

    /**
     * Check that the drivebase has been aligned for some acceptable ammount
     * of update iterations
     */
    public boolean hasBeenAligned() {
        return loopsThatDBIsAligned > 3;
    }

    @Override
    public final void update(final TorqueMode mode) {
        // *** LOG SOME STUFF TO SMART DASHBOARD ***
        Debug.log("Is Aligned", isAligned());
        Debug.log("Has Been Aligned", hasBeenAligned());
        Debug.log("Align Target", getAlignTarget());
        Debug.log("Drivebase State", desiredState.toString());
        Debug.log("Speed Setting at Start", speedSetting.toString());

        // offsetTargetingPID.setP(SmartDashboard.getNumber("Align PID P", 0));

        if ((shooter.wantsState(Shooter.State.SMART) || shooter.wantsState(Shooter.State.FUTURE_SMART_ALIGN)
                || shooter.wantsState(Shooter.State.LASER))
                // ^ these are the 3 states we want to align in
                && !shooter.isShift() && !shooter.inDebugMode()) {
            // ^ if we are in shift or debug mode then we dont want to align
            runSpeedSequence();
            desiredState = State.ALIGN_TO_ANGLE;
        } else if (wantsState(State.DASH)) {
            // dash forward at max velocity
            inputSpeeds = new TorqueSwerveSpeeds(MAX_VELOCITY, 0,
                    headingLockPID.calculate(perception.getHeading().getDegrees(), 15));
        } else {
            if (mode.isTeleop()) {
                desiredState = State.FIELD_RELATIVE;
            }
        }

        // Make drivebase slow during climb mode
        if (shooter.wantsState(Shooter.State.CLIMB)) {
            runSpeedSequence();
        }

        // If we are in FIELD_RELATIVE or ALIGN_TO_ANGLE then we want to convert our
        // field
        // relative chassis speeds into robot relative chassis speeds. We also want to
        // multiply by speed setting stuff.
        if (wantsState(State.FIELD_RELATIVE) || wantsState(State.ALIGN_TO_ANGLE) || wantsState(State.DASH)) {
            if (!wantsState(State.DASH)) { // If not dash then we apply speed settings.
                inputSpeeds = inputSpeeds
                        .times(speedSetting == SpeedSetting.SEQ ? speedSequence.get() : speedSetting.speed);
            }
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(perception.getHeading());
        }

        // We calculate fusedTargetOffset outside of the scope we need it for so
        // we can log it nicely.
        final Optional<Double> fusedTargetOffset = perception.getFusedTargetOffset();
        Debug.log("Target Offset Present", fusedTargetOffset.isPresent());
        Debug.log("Fused Target Offset", fusedTargetOffset.orElseGet(() -> 0.0));

        // If we are in the align state then we want to set our rotational velocity to
        // the output of the align to angle PID controller.
        if (wantsState(State.ALIGN_TO_ANGLE)) {
            double requestedAngularVelocity = 0;

            if (shooter.wantsState(Shooter.State.LASER) && !shooter.isShift()) {
                requestedAngularVelocity = headingLockPID.calculate(perception.getHeading().getDegrees(),
                        field.getAngleToLaser(perception.getPose()).getDegrees());
            } else if (fusedTargetOffset.isPresent()) {
                // We see the correct targets, we can lock our shooter to that target.
                requestedAngularVelocity = offsetTargetingPID.calculate(fusedTargetOffset.get(), 0);
            } else {
                // We do not see the correct targets, we need to lock to our estimated angle.
                requestedAngularVelocity = headingLockPID.calculate(perception.getHeading().getDegrees(),
                        getAlignTarget());
            }

            inputSpeeds.omegaRadiansPerSecond = -TorqueMath.constrain(requestedAngularVelocity, 2 * Math.PI);
        }

        // Handle alignment readiness counter. Increment every loop the drivebase is
        // aligned properly. Set back to zero if the drivebase is not aligned properly.
        // This makes the variable hold the ammount of consecutive iterations that
        // the drivebase has been properly alligned.
        //
        // We check if the drivebase is aligned properly by first checking which
        // alignment
        // mode we are in, wether its alignment mode or angle mode. Then we use that
        // mode
        // to check if we are aligned properly.
        if (fusedTargetOffset.isPresent() ? isTargetLocked(fusedTargetOffset.get()) : isAligned()) {
            loopsThatDBIsAligned++;
        } else {
            loopsThatDBIsAligned = 0;
        }

        // Use kinematics to convert robot vector to swerve vectors, then desaturate
        // such that some wheel is always using our proper full speed.
        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        // If there is no input velocity leave the modules as is.
        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleAngles(
                    swerveStates[0].angle,
                    swerveStates[1].angle,
                    swerveStates[2].angle,
                    swerveStates[3].angle);
            // Otherwise set the states to the desired module states.
        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }
    }

    /**
     * Set the swerve module's to a given rotation with 0 m/s translational
     * velocity.
     */
    private void manuallySetModuleAngles(final Rotation2d flAngle, final Rotation2d frAngle, final Rotation2d blAngle,
            final Rotation2d brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, flAngle));
        fr.setDesiredState(new SwerveModuleState(0, frAngle));
        bl.setDesiredState(new SwerveModuleState(0, blAngle));
        br.setDesiredState(new SwerveModuleState(0, brAngle));
    }

    /** Run the speed sequence deceleration */
    public void runSpeedSequence() {
        if (speedSetting != SpeedSetting.SEQ) {
            speedSequence = new SpeedSequence(Drivebase.SpeedSetting.FAST, Drivebase.SpeedSetting.SLOW, 1);
            speedSetting = SpeedSetting.SEQ;
        }
    }

    /**
     * Returns the current pose of the robot, used to satisfy the
     * TorquePathingDrivebase interface.
     */
    @Override
    public Pose2d getPose() {
        return perception.getPose();
    }

    /**
     * Sets the pose of the robot, used to satisfy the the TorquePathingDrivebase
     * interface.
     */
    @Override
    public void setPose(Pose2d pose) {
        perception.setPose(pose);
    }

    /**
     * Set the input speeds of the drivebase. This is actually an important setter.
     * But it also must be present to satisfy the the TorquePathingDrivebase
     * interface.
     */
    @Override
    public void setInputSpeeds(TorqueSwerveSpeeds speeds) {
        inputSpeeds = speeds;
    }

    /**
     * Get the current running chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return inputSpeeds;
    }

    /**
     * onBeginPathing() is called when the robot is about to start pathing.
     * onEndPathing() is called when the robot is about to end pathing
     * 
     * These are used for making sure that the drivebase knows when it
     * is following a path.
     */
    public void onBeginPathing() {
        setState(State.PATHING);
    }

    public void onEndPathing() {
        setState(State.FIELD_RELATIVE);
    }

   

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }

    public ChassisSpeeds getActualChassisSpeeds() {
        return kinematics.toChassisSpeeds(swerveStates);
    }
}