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
import edu.wpi.first.math.util.Units;

public final class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    public static boolean isPoseOnField(final Pose2d pose) {
        return TorqueMath.constrained(pose.getX(), 0, FIELD_LENGTH)
            && TorqueMath.constrained(pose.getY(), 0, FIELD_WIDTH);
    }

    public static boolean isIDValid(final int id) {
        return id <= 16;
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