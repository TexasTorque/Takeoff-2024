/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.function.Supplier;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022.SwerveConfig;
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

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State>
        implements Subsystems, TorquePathingDrivebase {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN_TO_ANGLE(ROBOT_RELATIVE),
        PATHING(null);
        // ^ PATHING is an extra state to be like ROBOT_RELATIVE but its explicity
        // pathing

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

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

    public static final double WIDTH = Units.inchesToMeters(21.25);

    public final static double MAX_VELOCITY = SwerveConfig.WHEEL_FREE_SPEED, MAX_ACCELERATION = 5,
            MAX_ANGULAR_VELOCITY = 2 * Math.PI, ANGULAR_VELOCITY_COEFFICIENT = 1;

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    public final SwerveDriveKinematics kinematics;

    private final TorqueSwerveModule2022 fl, fr, bl, br;

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public SpeedSequence speedSequence = new SpeedSequence(speedSetting, speedSetting, -1);

    private final PIDController alignPID;

    private double loopsThatDBIsAligned = 0;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        SwerveConfig swerveConfig = SwerveConfig.swervex;

        fl = new TorqueSwerveModule2022("Front Left", Ports.FL_MOD, swerveConfig, .2);
        fr = new TorqueSwerveModule2022("Front Right", Ports.FR_MOD, swerveConfig, .2);
        bl = new TorqueSwerveModule2022("Back Left", Ports.BL_MOD, swerveConfig, .2);
        br = new TorqueSwerveModule2022("Back Right", Ports.BR_MOD, swerveConfig, .2);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        alignPID = new PIDController(.1, 0, 0);
        alignPID.enableContinuousInput(0, 360);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        // Set the angle target for ALIGN_TO_ANGLE state, basically makes that state
        // an "align to goal" state.
        setAlignTarget(perception::getFilteredAngleToSpeaker);

        mode.onAuto(() -> {
            desiredState = State.FIELD_RELATIVE;
        });

        mode.onTeleop(() -> {
            desiredState = State.FIELD_RELATIVE;
        });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                fl.getPosition(), fr.getPosition(),
                bl.getPosition(), br.getPosition()
        };
    }

    private Supplier<Rotation2d> alignTarget = () -> Rotation2d.fromDegrees(0);

    public void setAlignTarget(final Rotation2d target) {
        alignTarget = () -> target;
    }

    public void setAlignTargetRelative(final Rotation2d target) {
        alignTarget = () -> target.plus(perception.getHeading());
    }

    public void setAlignTarget(final Supplier<Rotation2d> target) {
        alignTarget = target;
    }

    private double getAlignTarget() {
        return TorqueMath.constrain0to360(alignTarget.get().getDegrees());
    }

    public boolean isAligned() {
        return TorqueMath.toleranced(perception.getHeading().getDegrees(), getAlignTarget(), 2);
    }

    public boolean hasBeenAligned() {
        return loopsThatDBIsAligned > 15;
    }

    private boolean lockingOnToGoal = false;

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("Is Aligned", isAligned());
        Debug.log("Has Been Aligned", hasBeenAligned());
        Debug.log("Align Target", getAlignTarget());
        Debug.log("Drivebase State", desiredState.toString());

        // If shooter is in smart mode then the driver can still drive around but
        // the rotation should stay locked to the goal.
        if (shooter.wantsState(Shooter.State.SMART) && shooter.hasConsent() && !shooter.isShift() && mode.isTeleop() && !shooter.isDebugMode()) {
            desiredState = State.ALIGN_TO_ANGLE;
            // If we are not in the slowdown sequence speed setting
            if (speedSetting != SpeedSetting.SEQ) {
                // We gotta make sure we set it up
                speedSetting = SpeedSetting.SEQ;
                // If this is the first loop that we are locking onto the goal then we need
                // to create a new speed sequence. If we are already in the speed sequence
                // state during the first loop where we are locking onto the goal then the
                // driver was already in the speed sequence and we dont want to mess them up.
                if (!lockingOnToGoal) {
                    speedSequence = new SpeedSequence(Drivebase.SpeedSetting.FAST, Drivebase.SpeedSetting.SLOW, 1);
                }
            }
            lockingOnToGoal = true; // we are locking on
        } else {
            lockingOnToGoal = false; // we are not locking on
            if (mode.isTeleop())
                desiredState = State.FIELD_RELATIVE;
        }

        if (isAligned())
            loopsThatDBIsAligned++;
        else
            loopsThatDBIsAligned = 0;

        // If we are in FIELD_RELATIVE or ALIGN_TO_ANGLE then we want to convert our
        // field
        // relative chassis speeds into robot relative chassis speeds. We also want to
        // multiply by speed setting stuff.
        if (wantsState(State.FIELD_RELATIVE) || wantsState(State.ALIGN_TO_ANGLE)) {
            inputSpeeds = inputSpeeds.times(speedSetting == SpeedSetting.SEQ ? speedSequence.get()
                    : speedSetting.speed).toFieldRelativeSpeeds(perception.getHeading());
        }

        // If we are in the align state then we want to set our rotational velocity to
        // the output of the align to angle PID controller.
        if (wantsState(State.ALIGN_TO_ANGLE)) {
            inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(
                    alignPID.calculate(perception.getHeading().getDegrees(), getAlignTarget()), Math.PI);
        }

        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleStates(swerveStates[0].angle,
                    swerveStates[1].angle, swerveStates[2].angle,
                    swerveStates[3].angle);
        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }
    }

    private void manuallySetModuleStates(final Rotation2d flAngle, final Rotation2d frAngle, final Rotation2d blAngle,
            final Rotation2d brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, flAngle));
        fr.setDesiredState(new SwerveModuleState(0, frAngle));
        bl.setDesiredState(new SwerveModuleState(0, blAngle));
        br.setDesiredState(new SwerveModuleState(0, brAngle));
    }

    @Override
    public Pose2d getPose() {
        return perception.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        perception.setPose(pose);
    }

    @Override
    public void setInputSpeeds(TorqueSwerveSpeeds speeds) {
        inputSpeeds = speeds;
    }

    public void setInputSpeedsTeleop(TorqueSwerveSpeeds speeds) {
        inputSpeeds = speeds;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return inputSpeeds;
    }

    public void onBeginPathing() {
        setState(State.PATHING);
    }

    public void onEndPathing() {
        setState(State.FIELD_RELATIVE);
    }

    public double getRadius() {
        return WIDTH * Math.sqrt(2);
    }

    @Override
    public void clean(TorqueMode mode) {
        if (mode.isTeleop()) {
            desiredState = desiredState.parent;
        }
    }
}