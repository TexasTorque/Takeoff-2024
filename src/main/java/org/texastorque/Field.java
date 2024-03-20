/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
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

/**
 * Holding information about the field.
 */
public final class Field {
    private static volatile Field instance;

    public static final double LENGTH = 16.541;
    public static final double WIDTH = Units.inchesToMeters(315.5);
    public static final double SPEAKER_Y = 5.55;
    public static final double SPEAKER_X = 0;

    public Pose2d speakerPose = new Pose2d();
    public Pose2d passingZone = new Pose2d();

    public boolean isRedAlliance;

    public Field() {
        SmartDashboard.putNumber("Speaker Y Position", 5.55);
        SmartDashboard.putNumber("Speaker X Position", 0);
    }
    /**
     * Update the alliance information information.
     */
    public void updateAlliance() {
        isRedAlliance = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        speakerPose = new Pose2d(isRedAlliance ? SPEAKER_X : LENGTH - SPEAKER_X, 
                SPEAKER_Y, Rotation2d.fromDegrees(0));
    }

    public boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, LENGTH)
                && TorqueMath.constrained(pose.getY(), 0, WIDTH);
    }

    /** Check if an apriltag id is valid, id elementof [1, 16] */
    public boolean isIDValid(final int id) {
        return 3 <= id && id <= 8;
    }

    /** Compute the distance between two poses */
    public final double distanceBetween(final Pose2d pose1, final Pose2d pose2) {
        return Math.sqrt(
                Math.pow(pose1.getY() - pose2.getY(), 2)
                        + Math.pow(pose1.getX() - pose2.getX(), 2));
    }

    /** Calculates the distance from some pose to the speaker */
    public final double distanceToSpeaker(final Pose2d pose) {
        return distanceBetween(speakerPose, pose);
    }

    /** Compute angle between two poses */
    public Rotation2d angleBetween(final Pose2d constant, final Pose2d bot) {
        return Rotation2d.fromRadians(
                Math.atan2(constant.getY() - bot.getY(), constant.getX() - bot.getX()));
    }

    /** Make a rotation intake relative -- varies based on alliance */
    public Rotation2d intakeRelative(final Rotation2d angle) {
        return angle.plus(Rotation2d.fromRadians(isRedAlliance ? Math.PI : 0));
    }

    /** Make a rotation shooter relative -- varies based on alliance */
    public Rotation2d shooterRelative(final Rotation2d angle) {
        return angle.plus(Rotation2d.fromRadians(isRedAlliance ? 0 : Math.PI));
    }

    /** Calculates the angle from some pose to the passing zone */
    public Rotation2d getAngleToPassingZone(final Pose2d pose) {
        return shooterRelative(angleBetween(pose, passingZone));
    }

    /** Calculate the angle from some pose to the speaker */
    public Rotation2d getAngleToSpeaker(final Pose2d pose) {
        return shooterRelative(angleBetween(pose, speakerPose));
    }

    /** Is some pose's X coord > xPosition away from the current alliance wall */
    public boolean isXPast(final Pose2d pose, final double xPosition) {
        return !isRedAlliance ? pose.getX() > xPosition : pose.getX() < LENGTH - xPosition;
    }


    /** Handling x-offseting correctly using the alliances color */
    public Pose2d calculateXOffset(final Pose2d pathEndPosition, final double xOffset) {
        return new Pose2d(pathEndPosition.getX() + (isRedAlliance ? -xOffset : xOffset), pathEndPosition.getY(),
                pathEndPosition.getRotation());
    }

    /** Load the apriltag field layout from reasources */
    public AprilTagFieldLayout getFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    /** Reflect some pose based on our current alliance position */
    public Pose2d getAllianceReflectedPose(Pose2d pose) {
        return new Pose2d(isRedAlliance ? LENGTH - pose.getX() : pose.getX(), pose.getY(), pose.getRotation());
    }

    public int getSpeakerTargetID() {
        return isRedAlliance ? 4 : 7;
    }

    public static synchronized final Field getInstance() {
        return instance == null ? instance = new Field() : instance;
    }
}