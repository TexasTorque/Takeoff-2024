/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import java.io.IOException;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Field {
    private static volatile Field instance;

    public static final double LENGTH = Units.inchesToMeters(651.25);
    public static final double WIDTH = Units.inchesToMeters(315.5);

    public Pose2d SPEAKER_POSE_DISTANCE = new Pose2d();
    public Pose2d SPEAKER_POSE_ANGLE_RIGHT = new Pose2d();
    public Pose2d SPEAKER_POSE_ANGLE_LEFT = new Pose2d();
    public boolean isRedAlliance;

    public Field() {
        SmartDashboard.putNumber("Speaker Angle Threshold", 3);
    }

    public void updateAlliance() {
        isRedAlliance = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        double speakerPosition = isRedAlliance ? LENGTH : 0;

        SPEAKER_POSE_DISTANCE = new Pose2d(-.04 * (isRedAlliance ? -1 : 1) + speakerPosition, 5.55,
                Rotation2d.fromDegrees(0));

        SPEAKER_POSE_ANGLE_RIGHT = new Pose2d(speakerPosition, 6., Rotation2d.fromDegrees(0));
        SPEAKER_POSE_ANGLE_LEFT = new Pose2d(speakerPosition, 5.6, Rotation2d.fromDegrees(0));
    }

    public boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, LENGTH)
                && TorqueMath.constrained(pose.getY(), 0, WIDTH);
    }

    public boolean isIDValid(final int id) {
        return id <= 16;
    }

    public Rotation2d getAngleToSpeaker(final Pose2d pose) {
        double position = SmartDashboard.getNumber("Speaker Angle Threshold", 0);
        return Rotation2d.fromRadians(Math.atan2(
                (pose.getY() < position ? SPEAKER_POSE_ANGLE_RIGHT.getY() : SPEAKER_POSE_ANGLE_LEFT.getY()) - pose.getY(),
                SPEAKER_POSE_ANGLE_RIGHT.getX() - pose.getX()))
                .plus(Rotation2d.fromRadians(isRedAlliance ? 0 : Math.PI));
    }

    public boolean isXPast(final Pose2d pose, final double xPosition) {
        return !isRedAlliance ? pose.getX() > xPosition : pose.getX() < LENGTH - xPosition;
    }

    public Pose2d getEndPosition(final Pose2d pathEndPosition, final boolean isCenterLine) {
        if (!isCenterLine)
            return pathEndPosition;
        return new Pose2d(pathEndPosition.getX() + .5 * (isRedAlliance ? 1 : -1), pathEndPosition.getY(),
                pathEndPosition.getRotation());
    }

    public AprilTagFieldLayout getFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static synchronized final Field getInstance() {
        return instance == null ? instance = new Field() : instance;
    }
}