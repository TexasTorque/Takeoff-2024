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
    public static final double REAL_SPEAKER_Y_POSE = 5.55, FIELD_Y_BOUNDRY = 3.0;

    public Pose2d speakerPoseDistance = new Pose2d();
    public Pose2d speakerPoseAngle = new Pose2d();
    public Pose2d speakerPoseAngleLeft = new Pose2d();

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


        final double speakerXPosition = isRedAlliance ? LENGTH : 0;

        speakerPoseDistance = new Pose2d(-.04 * (isRedAlliance ? -1 : 1) + speakerXPosition,
                5.55, Rotation2d.fromDegrees(0));

        double speakerXPositionForAngle = .1;

        if (isRedAlliance)
            speakerXPositionForAngle = LENGTH - .1;

        speakerPoseAngle = new Pose2d(speakerXPositionForAngle, 5.55, Rotation2d.fromDegrees(0));
    }

    public boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, LENGTH)
                && TorqueMath.constrained(pose.getY(), 0, WIDTH);
    }

    /** Check if an apriltag id is valid, id elementof [1, 16] */
    public boolean isIDValid(final int id) {
        return 3 <= id && id <= 8;
    }

    /** Calculate the angle from some pose to the speaker */
    public Rotation2d getAngleToSpeaker(final Pose2d pose) {
        // final double speakerYPosition = SmartDashboard.getNumber("Speaker Y Position", 0);
        // final double speakerXPosition = SmartDashboard.getNumber("Speaker X Position", 0);
        // speakerPoseAngle = new Pose2d(speakerXPosition, speakerYPosition, Rotation2d.fromDegrees(0));

        return Rotation2d.fromRadians(
                Math.atan2(speakerPoseAngle.getY() - pose.getY(), speakerPoseAngle.getX() - pose.getX()))
                .plus(Rotation2d.fromRadians(isRedAlliance ? 0 : Math.PI));
    }

    /** Is some pose's X coord > xPosition away from the current alliance wall */
    public boolean isXPast(final Pose2d pose, final double xPosition) {
        return !isRedAlliance ? pose.getX() > xPosition : pose.getX() < LENGTH - xPosition;
    }

    /**
     * Get the end position handling x-offseting correctly using the alliances color
     * 
     * @deprecated use calculateXOffset instead
     */
    @Deprecated
    public Pose2d getEndPosition(final Pose2d pathEndPosition, final boolean isCenterLine) {
        if (!isCenterLine)
            return pathEndPosition;
        return new Pose2d(pathEndPosition.getX() + .5 * (isRedAlliance ? 1 : -1), pathEndPosition.getY(),
                pathEndPosition.getRotation());
        // This method needs to be changed back to having a configurable X-offset
        // Edit: that is the overload below
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

    public static synchronized final Field getInstance() {
        return instance == null ? instance = new Field() : instance;
    }
}