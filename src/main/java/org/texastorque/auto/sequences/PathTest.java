package org.texastorque.auto.sequences;

import java.util.ArrayList;
import java.util.List;

import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Util;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathTest extends TorqueSequence implements Subsystems {
    public static final PathConstraints PATH_CONST = new PathConstraints(1, 1, Math.PI, Math.PI);

    public PathPoint createPoint(final Pose2d pose) {
        return createPoint(pose.getTranslation(), pose.getRotation());
    }

    public PathPoint createPoint(final Translation2d trl, final Rotation2d rot) {
        final RotationTarget target = new RotationTarget(0, rot);
        return new PathPoint(trl, target, PATH_CONST);
    }

    public GoalEndState endState(List<PathPoint> points) {
        return new GoalEndState(0, points.get(points.size() - 1).rotationTarget.getTarget());
    }

    public PathTest() {
        final Pose2d currentPose = new Pose2d(1.1, 5.75, Field.ROT_FWD);

        final Pose2d notePose = Field.getNotePose(2);

        List<PathPoint> points = new ArrayList<>();

        points.add(createPoint(currentPose));
        points.add(createPoint(notePose.getTranslation(), Field.ROT_FWD));

        for (int i = 0; i < points.size(); i++)
            Debug.log("Point " + i,
                    Util.pose2d2str(new Pose2d(points.get(i).position, points.get(i).rotationTarget.getTarget())));

        // addBlock(new TorqueFollowPath(PathPlannerPath.fromPathPoints(points, PATH_CONST, endState(points)), drivebase, new ChassisSpeeds(), perception.getHeading(), 2));
        addBlock(new TorqueFollowPath("test", drivebase, new ChassisSpeeds(), perception.getHeading(), 2));
    }
}
