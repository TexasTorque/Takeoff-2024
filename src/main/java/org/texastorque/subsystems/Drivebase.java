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
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleX.SwerveConfig;
import org.texastorque.torquelib.swerve.TorqueSwerveModuleX;
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

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State>
        implements Subsystems, TorquePathingDrivebase {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), ALIGN_TO_ANGLE(ROBOT_RELATIVE);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    public enum SpeedSetting {
        SLOW(.25), MID(.5), FAST(1.0);

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

    private static volatile Drivebase instance;

    public static final double WIDTH = Units.inchesToMeters(58 / 3);

    public final static double MAX_VELOCITY_TELEOP = 4.6, MAX_ACCELERATION = 2,
            MAX_ANGULAR_VELOCITY = 6, ANGULAR_VELOCITY_COEFFICIENT = .05;

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    public final SwerveDriveKinematics kinematics;

    private final TorqueSwerveModuleX fl, fr, bl, br;

    private final PIDController teleopOmegaController;

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    private final PIDController alignPID;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        final SwerveConfig swerveConfig = SwerveConfig.defaultConfig;

        swerveConfig.driveGearRatio = 2;

        fl = new TorqueSwerveModuleX("Front Left", Ports.FL_MOD, swerveConfig);
        fr = new TorqueSwerveModuleX("Front Right", Ports.FR_MOD, swerveConfig);
        bl = new TorqueSwerveModuleX("Back Left", Ports.BL_MOD, swerveConfig);
        br = new TorqueSwerveModuleX("Back Right", Ports.BR_MOD, swerveConfig);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        alignPID = new PIDController(.15, 0, 0);
        alignPID.enableContinuousInput(0, 360);

        teleopOmegaController = new PIDController(.5 * Math.PI, 0, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> {
            desiredState = State.ROBOT_RELATIVE;
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
        return TorqueMath.toleranced(perception.getHeading().getDegrees(),
                getAlignTarget(), 5);
    }

    public boolean isRotationLocked = true;

    @Override
    public final void update(final TorqueMode mode) {
        if (mode.isTeleop()) {
            // correctHeading();
            inputSpeeds = inputSpeeds
                    .toFieldRelativeSpeeds(perception.getHeading());
            // .plus(perception.getAngularVelocity().times(ANGULAR_VELOCITY_COEFFICIENT)))
        }

        if (wantsState(State.ALIGN_TO_ANGLE)) {
            inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(
                    alignPID.calculate(perception.getHeading().getDegrees(), getAlignTarget()), .75);
        }

        swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY_TELEOP);

        if (inputSpeeds.hasZeroVelocity()) {
            manuallySetModuleStates(swerveStates[0].angle.getRadians(),
                    swerveStates[1].angle.getRadians(), swerveStates[2].angle.getRadians(),
                    swerveStates[3].angle.getRadians());

        } else {
            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);
        }

        if (mode.isTeleop())
            desiredState = desiredState.parent;

    }

    private void manuallySetModuleStates(final double flAngle, final double frAngle,
            final double blAngle, final double brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(flAngle)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(frAngle)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(blAngle)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(brAngle)));
    }

    double lastRotationRadians = 0;

    public void correctHeading() {
        final double realRotationRadians = perception.getHeading().getRadians();

        if (isRotationLocked && !inputSpeeds.hasRotationalVelocity() && inputSpeeds.hasTranslationalVelocity()) {
            final double omega = teleopOmegaController.calculate(realRotationRadians, lastRotationRadians);
            inputSpeeds.omegaRadiansPerSecond = omega;
        } else
            lastRotationRadians = realRotationRadians;
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

}