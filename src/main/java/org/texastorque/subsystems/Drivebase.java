/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveX;
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



    // Speed shifter != state for drivebase 
    public enum State implements TorqueState {
        SLOW(.25), MID(.5), FAST(1.0);

        private static final State[] vals = values();

        public double speed;

        private State(final double speed) {
            this.speed = speed;
        }

        public State shiftUp() {
            return vals[Math.min((this.ordinal() + 1), vals.length - 2)];
        }

        public State shiftDown() {
            return vals[Math.max((this.ordinal() - 1), 0)];
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = Units.inchesToMeters(58 / 3), MAX_VELOCITY_TELEOP = 4.6,
            MAX_ANGULAR_VELOCITY = 2 * Math.PI,
            ANGULAR_VELOCITY_COEFFICIENT = .085;

    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    public final SwerveDriveKinematics kinematics;

    private final TorqueSwerveX fl, fr, bl, br;

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds;

    public Drivebase() {
        super(State.FAST);

        fl = new TorqueSwerveX("Front Left", Ports.FL_MOD);
        fr = new TorqueSwerveX("Front Right", Ports.FR_MOD);
        bl = new TorqueSwerveX("Back Left", Ports.BL_MOD);
        br = new TorqueSwerveX("Back Right", Ports.BR_MOD);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        swerveStates = new SwerveModuleState[4];

        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    @Override
    public final void update(final TorqueMode mode) {
        if (mode.isTeleop())
            inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(perception.getHeading()
                    .plus(perception.getAngularVelocity().times(ANGULAR_VELOCITY_COEFFICIENT)))
                    .times(desiredState.speed);

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
    }

    private void manuallySetModuleStates(final double flAngle, final double frAngle,
            final double blAngle, final double brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(flAngle)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(frAngle)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(blAngle)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(brAngle)));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                fl.getPosition(), fr.getPosition(),
                bl.getPosition(), br.getPosition()
        };
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

    public ChassisSpeeds getChassisSpeeds() {
        return inputSpeeds;
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}
