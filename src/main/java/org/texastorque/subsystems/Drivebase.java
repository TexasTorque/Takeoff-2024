/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;
import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveX;
import org.texastorque.torquelib.util.TorqueMath;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
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

    public static final Pose2d INITIAL_POS = new Pose2d(0, 0, Rotation2d.fromRadians(0));

    public final static double MAX_VELOCITY_TELEOP = 4.6, MAX_ACCELERATION = 2,
            MAX_ANGULAR_VELOCITY = 6;

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    public final SwerveDriveKinematics kinematics;

    private final TorqueSwerveX fl, fr, bl, br;

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public double ANGULAR_VELOCITY_COEFFICIENT = .05;

    private final PIDController alignPID;

    public boolean flag1 = false;

    List<Translation2d> bezierPoints;

    PathPlannerPath path;

    private final PPHolonomicDriveController controller;

    private final PIDConstants translationConstants = new PIDConstants(1, 0, 0);
    private final PIDConstants rotationConstants = new PIDConstants(Math.PI * 2.5, 0, 0);
    private PathPlannerTrajectory trajectory;

    private final Timer pathTimer = new Timer();

    private final PIDController teleopOmegaController = new PIDController(.5 * Math.PI, 0, 0);

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveX("Front Left", Ports.FL_MOD);
        fr = new TorqueSwerveX("Front Right", Ports.FR_MOD);
        bl = new TorqueSwerveX("Back Left", Ports.BL_MOD);
        br = new TorqueSwerveX("Back Right", Ports.BR_MOD);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        Debug.log("Angular Velocity Coeff", ANGULAR_VELOCITY_COEFFICIENT);

        alignPID = new PIDController(.15, 0, 0);
        alignPID.enableContinuousInput(0, 360);

        controller = new PPHolonomicDriveController(translationConstants, rotationConstants, 3,
                WIDTH / 2);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> {
            desiredState = State.ROBOT_RELATIVE;
        });

        mode.onTeleop(() -> {
            desiredState = State.FIELD_RELATIVE;
            flag1 = false;
        });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { 
            fl.getPosition(), fr.getPosition(),
            bl.getPosition(), br.getPosition() 
        };
    }

    private DoubleSupplier alignTarget = () -> 0;

    public void setAlignTarget(final double target) {
        alignTarget = () -> target;
    }

    public void setAlignTarget(final DoubleSupplier target) {
        alignTarget = target;
    }

    private double getAlignTarget() {
        return TorqueMath.constrain0to360(alignTarget.getAsDouble());
    }

    public boolean isAligned() {
        return TorqueMath.toleranced(perception.getHeading().getDegrees(),
                getAlignTarget(), 5);
    }

    public boolean isRotationLocked = true;

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("flag1", flag1);

        if (mode.isTeleop()) {
            // correctHeading();
            inputSpeeds = inputSpeeds
                    .toFieldRelativeSpeeds(perception.getHeading());
            // .plus(perception.getAngularVelocity().times(ANGULAR_VELOCITY_COEFFICIENT)))
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
