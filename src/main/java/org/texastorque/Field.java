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

public final class Field {
    private static volatile Field instance;

    public static final double LENGTH = Units.inchesToMeters(651.25);
    public static final double WIDTH = Units.inchesToMeters(315.5);

    public static final Rotation2d ROT_FWD = Rotation2d.fromDegrees(0);
    public static final Rotation2d ROT_BACK = Rotation2d.fromDegrees(180);

    public final Pose2d SPEAKER_POSE_ANGLE_RIGHT;
    public final Pose2d SPEAKER_POSE_ANGLE_LEFT;
    public final Pose2d SPEAKER_POSE_DISTANCE;

    public Field() {
        boolean isRedAlliance = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        double speakerPosition = isRedAlliance ? LENGTH : 0;

        SPEAKER_POSE_ANGLE_RIGHT = new Pose2d(speakerPosition, 6.5, Rotation2d.fromDegrees(0));
        SPEAKER_POSE_ANGLE_LEFT = new Pose2d(speakerPosition, 6, Rotation2d.fromDegrees(0));
        SPEAKER_POSE_DISTANCE = new Pose2d(-.04 * (isRedAlliance ? -1 : 1) + speakerPosition, 5.55,
                Rotation2d.fromDegrees(0));
    }

    public boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, LENGTH)
                && TorqueMath.constrained(pose.getY(), 0, WIDTH);
    }

    public boolean isIDValid(final int id) {
        return id <= 16;
    }

    /**
     * Get the angle from pose to the speaker alliance respective.
     */
    public Rotation2d getAngleToSpeakerAlliance(final Pose2d pose) {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return Rotation2d.fromRadians(Math.PI * 2).minus(getAngleToSpeaker(pose));
        } else
            return getAngleToSpeaker(pose);
    }

    public Rotation2d getAngleToSpeaker(final Pose2d pose) {
        return Rotation2d.fromRadians(Math.atan2(
                (pose.getY() < 4.5 ? SPEAKER_POSE_ANGLE_RIGHT.getY() : SPEAKER_POSE_ANGLE_LEFT.getY())
                        - pose.getY(),
                SPEAKER_POSE_ANGLE_RIGHT.getX() - pose.getX())).plus(Rotation2d.fromRadians(Math.PI));
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