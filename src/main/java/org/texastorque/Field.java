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

public final class Field {
    public static final double LENGTH = Units.inchesToMeters(651.25);
    public static final double WIDTH = Units.inchesToMeters(315.5);

    public static final Rotation2d ROT_FWD = Rotation2d.fromDegrees(0);
    public static final Rotation2d ROT_BACK = Rotation2d.fromDegrees(180);

    public static boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, LENGTH)
                && TorqueMath.constrained(pose.getY(), 0, WIDTH);
    }

    public static boolean isIDValid(final int id) {
        return id <= 16;
    }

    public static final Pose2d SPEAKER_POSE_ANGLE = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d SPEAKER_POSE_DISTANCE = new Pose2d(-.04, 5.55, Rotation2d.fromDegrees(0));

    /**
     * Get the angle from pose to the speaker.
     */
    public static Rotation2d getAngleToSpeaker(final Pose2d pose) {
        return Rotation2d.fromRadians(Math.atan2(
                Field.SPEAKER_POSE_ANGLE.getY() - pose.getY(),
                Field.SPEAKER_POSE_ANGLE.getX() - pose.getX())).plus(Rotation2d.fromRadians(Math.PI));
    }

    public static AprilTagFieldLayout getFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
}