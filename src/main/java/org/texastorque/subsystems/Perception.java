package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Camera;
import org.texastorque.toast.lib.Toast;
import org.texastorque.toast.lib.Util;
import org.texastorque.toast.lib.pipelines.AprilTags;
import org.texastorque.toast.lib.pipelines.ObjDetector;
import org.texastorque.toast.lib.pipelines.AprilTags.AprilTagDetection;
import org.texastorque.toast.lib.pipelines.ObjDetector.Detectable;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import com.fasterxml.jackson.databind.JsonNode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Robot perception subsystem, handles sensors that the robot uses
 * to contextualize it's surroundings.
 * 
 * 
 * Other perception subsystems are below for reference:
 * -
 * https://github.com/TexasTorque/Bravo-2024/blob/21ab2550dfbc8c3d1c73813b9feec5aa52dac205/src/main/java/org/texastorque/subsystems/Perception.java
 * -
 * https://github.com/TexasTorque/Banana-2024/blob/0d04f9470ef629f2e7fc1128fed3fcafd0b1d2bf/src/main/java/org/texastorque/subsystems/Perception.java
 * 
 * @author Justus
 */
public final class Perception extends TorqueStatorSubsystem<Perception.State> implements Subsystems {

    public enum State implements TorqueState {
        VISION;
    }

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in meters and radians, then meters.
     */
    private static final Vector<N3> ODOMETRY_STDS = VecBuilder.fill(.05, .05, Units.degreesToRadians(2));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */

    private static final Vector<N3> VISION_STDS = VecBuilder.fill(.1, .1, Units.degreesToRadians(5));

    /**
     * The maximum angular velocity of the robot (in radians per second) and maximum
     * distance from
     * the camera to the april tag (in meters) where we trust the vision
     * measurements.
     */
    private static final double MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2, MAX_DISTANCE = 6;

    private final Toast toast;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();
    public final Field2d field = new Field2d();
    private AprilTagFieldLayout fieldMap = Field.getFieldLayout();

    public Perception() {
        super(State.VISION);

        toast = new Toast();
        poseEstimator = new SwerveDrivePoseEstimator(
                drivebase.kinematics,
                getHeading(),
                drivebase.getModulePositions(),
                new Pose2d(), ODOMETRY_STDS, VISION_STDS);

        // Add toast cameras
        toast.addCamera(new Camera("SHTR_R", new Transform3d()));
        toast.addCamera(new Camera("SHTR_L", new Transform3d()));
        toast.addCamera(new Camera("INTK_R", new Transform3d()));
        toast.addCamera(new Camera("INTK_L", new Transform3d()));

        // Register the apriltags pipeline on all cameras
        toast.iterCams(cam -> cam.addPipeline(new AprilTags(cam.id)));

        // Register the object detection pipelines on intake cameras and configure them
        // to detect notes
        toast.getCamera("INTK_R").get().addPipeline(new ObjDetector<Note>(Note::fromJSONRight));
        toast.getCamera("INTK_L").get().addPipeline(new ObjDetector<Note>(Note::fromJSONLeft));

        // Log the field map to the dashboard
        Debug.field("Field", field);
    }

    @Override
    public void initialize(final TorqueMode mode) {
    }

    @Override
    public void update(final TorqueMode mode) {
        updateOdometryLocalization();
        updateVisionLocalization();

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        Debug.log("Pose", Util.pose2d2str(poseEstimator.getEstimatedPosition()));
        Debug.log("Heading (°)", getHeading().getDegrees());
    }

    public void updateOdometryLocalization() {
        // Updates the pose estimator with swerve encoder feedback
        poseEstimator.update(getHeading(), drivebase.getModulePositions());
    }

    private final Map<Integer, Pose3d> tagsInView = new HashMap<>();

    public void updateVisionLocalization() {
        toast.update(); // Updates all the vision pipelines.

        toast.iterCams((cam) -> {
            final var pipeOpt = cam.getPipeline(AprilTags.class);
            if (pipeOpt.isEmpty())
                return;
            final AprilTags pipe = pipeOpt.get();

            final List<AprilTagDetection> detections = pipe.getDetections();

            for (final AprilTagDetection detection : detections) {

                // if
                // - the detection is not "valid"
                // - the id is not an id on the field
                // - the robot is rotating too fast
                // - or the tag is too far away
                // then we ignore the detection and move on
                if (!detection.isValidDetection()
                        || !Field.isIDValid(detection.id)
                        || Math.abs(gyro.getAngularVelocity().getRadians()) > MAX_ANGULAR_VELOCITY_RADS
                        || detection.getDistance() > MAX_DISTANCE)
                    continue;

                // get tag pose in world space
                final Pose3d tagPose = fieldMap.getTagPose(detection.id).get(); // should never fail

                // we see this tag so we add it to the tagsInView map. this is so we can log the
                // detections on advantagescope
                if (!tagsInView.containsKey(detection.id))
                    tagsInView.put(detection.id, tagPose);

                // converting from cam space to robot space by adding the camera->tag transform
                // with the center->camera transform
                final Transform3d robotSpaceTransform = detection.transform.plus(cam.transform);

                // converting robot space to world space using the position of the tag
                final Pose3d estPose3d = tagPose.transformBy(robotSpaceTransform);

                final Pose2d estPose = estPose3d.toPose2d();

                // if the estimated position is off the field then something is wrong and we
                // must move on
                if (!Field.isPoseOnField(estPose))
                    continue;

                // add the processed vision messurement to the pose estimator
                poseEstimator.addVisionMeasurement(estPose, detection.timestamp);
            }
        });

        // Serializes and pushes the seen tags to networktables so we can view
        // detections on advantagescop
    }

    /**
     * Gyro heading (yaw, CCW around the Z-axis) as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return gyro.getHeadingCCW();
    }

    /**
     * Gyro angluar velocity (yaw, CCW around the Z-axis) as a Rotation2d.
     * Represents the angle (radians or degrees) per second.
     */
    public Rotation2d getAngularVelocity() {
        return gyro.getAngularVelocity();
    }

    /**
     * Tear the gyro, make the current heading "north" (0° yaw).
     */
    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
    }

    /**
     * Returns the current perceived position of the robot.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current position of the robot to the argument.
     */
    public void setPose(final Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), drivebase.getModulePositions(), pose);
    }

    /**
     * Reset the position in the pose estimator to be at the origin..
     */
    public void resetPose() {
        setPose(new Pose2d());
    }

    /**
     * Get the angle from the robot to the speaker.
     */
    public Rotation2d getAngleToSpeaker() {
        return Field.getAngleToSpeaker(getPose());
    }

    /**
     * Get the distance from the robot to the speaker
     */
    public double getDistanceToSpeaker() {
        return Math.sqrt(
                Math.pow(Field.SPEAKER_POSE.getY() - getPose().getY(), 2)
                        + Math.pow(Field.SPEAKER_POSE.getX() - getPose().getX(), 2));
    }

    public boolean isAboveSpeakerOnY() {
        return getPose().getY() > Field.SPEAKER_POSE.getY();
    }

    private static volatile Perception instance;

    public static synchronized final Perception getInstance() {
        return instance == null ? instance = new Perception() : instance;
    }

    public List<Note> getNoteDetections() {
        final List<Note> detRight = toast.getCamera("INKT_R").get().getPipeline(ObjDetector.class).get()
                .getDetections();
        final List<Note> detLeft = toast.getCamera("INKT_L").get().getPipeline(ObjDetector.class).get().getDetections();
        detRight.addAll(detLeft);
        return detRight;
    }

    public Optional<Note> getBestDetection() {
        return ObjDetector.getBestDetection(getNoteDetections());
    }

    /**
     * Class representing a Note detected by an ObjDetector pipeline.
     */
    public static class Note extends Detectable {

        public static final Note EMPTY = new Note("empty", 0, 0, 0);

        public static final double F = 70, W = 640, D = 100;

        public final String name;
        public final double x;
        public final double angle;

        private static double calculateAngle(final double x) {
            return (x / (2 * W * D) - 0.5) * (2 * F - (D * F) / W);
        }

        private static Note parseJSON(final JsonNode det) {
            final String name = det.get("name").asText("unknown");
            final double x = det.get("ctr_x").asDouble(0);
            final double confidence = det.get("conf").asDouble(0);
            return new Note(name, x, 180, confidence);
        }

        public static Note fromJSONRight(final JsonNode det) {
            final Note parsed = parseJSON(det);
            final double angle = calculateAngle(parsed.x + W - D);
            return new Note(parsed.name, parsed.x, angle, parsed.confidence);
        }

        public static Note fromJSONLeft(final JsonNode det) {
            final Note parsed = parseJSON(det);
            final double angle = calculateAngle(parsed.x);
            return new Note(parsed.name, parsed.x, angle, parsed.confidence);
        }

        public Note(final String name, final double x, final double angle, final double confidence) {
            this.name = name;
            this.x = x;
            this.angle = angle;
            this.confidence = confidence;
        }

        public String toString() {
            return String.format("%s @ %.2f° (%.2f conf)", name, angle, confidence);
        }
    }

    @Override
    public void clean(TorqueMode mode) {
    }

}
