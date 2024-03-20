/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.texastorque.Robot;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Camera;
import org.texastorque.toast.lib.Toast;
import org.texastorque.toast.lib.Util;
import org.texastorque.toast.lib.Pipeline.Status;
import org.texastorque.toast.lib.pipelines.AprilTags;
import org.texastorque.toast.lib.pipelines.ObjDetector;
import org.texastorque.toast.lib.pipelines.AprilTags.AprilTagDetection;
import org.texastorque.toast.lib.pipelines.ObjDetector.Detectable;
import org.texastorque.torquelib.Debug;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueRollingMedian;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import com.fasterxml.jackson.databind.JsonNode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Robot perception subsystem, handles sensors that the robot uses
 * to contextualize it's surroundings.
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

    private static final Vector<N3> VISION_STDS = VecBuilder.fill(.1, .1, Units.degreesToRadians(1));

    /**
     * The maximum angular velocity of the robot (in radians per second) and maximum
     * distance from
     * the camera to the april tag (in meters) where we trust the vision
     * measurements.
     */
    private static final double MAX_ANGULAR_VELOCITY_RADS = Math.PI * 2, MAX_DISTANCE = 6;

    // Toast handles camera interfacing, poseEstimator is used to aggregate
    // pose estimations and find a true pose estimate
    private final Toast toast;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();
    public final Field2d field2d = new Field2d();

    private final AprilTagFieldLayout fieldMap; // local instance of the field layout

    // Used to filter some noise directly out of the pose measurements.
    private final TorqueRollingMedian filteredX, filteredY;
    private Pose2d filteredPose = new Pose2d();

    // a position that we could be at in the future that we want to
    // run computations for.
    private Pose2d futureShootingPose = new Pose2d();

    public static final String SHTR_R = "SHTR_R", SHTR_L = "SHTR_L", INTK_R = "INTK_R";

    private final AprilTags tagCameraLeft, tagCameraRight;
    private final ObjDetector<Note> intakeCamera;

    @SuppressWarnings("unchecked")
    public Perception() {
        super(State.VISION);

        toast = new Toast();
        poseEstimator = new SwerveDrivePoseEstimator(
                drivebase.kinematics,
                getHeading(),
                drivebase.getModulePositions(),
                new Pose2d(), ODOMETRY_STDS, VISION_STDS);

        // Add toast camera -- question: does pitch need to be here?
        toast.addCamera(new Camera(SHTR_R,
                Camera.transformInchDeg(-5.0, 12.54, 14.181, 0, 35, 180)));
        toast.addCamera(new Camera(SHTR_L,
                Camera.transformInchDeg(-5.0, -12.54, 14.181, 0, 35, 180)));
        toast.addCamera(new Camera(INTK_R, new Transform3d()));

        // Register the apriltags pipeline on shooter cameras
        tagCameraLeft = (AprilTags) toast.getCamera(SHTR_L).get().addPipeline(new AprilTags());
        tagCameraRight = (AprilTags) toast.getCamera(SHTR_R).get().addPipeline(new AprilTags());

        // Register the object detection pipelines on intake cameras and configure them
        // to detect notes
        intakeCamera = (ObjDetector<Note>) toast.getCamera(INTK_R).get()
                .addPipeline(new ObjDetector<Note>(Note::fromJSONRight));

        // Log the field map to the dashboard
        Debug.field("Field", field2d);

        filteredX = new TorqueRollingMedian(5);
        filteredY = new TorqueRollingMedian(5);

        fieldMap = field.getFieldLayout();
    }

    @Override
    public void initialize(final TorqueMode mode) {
    }

    @Override
    public void update(final TorqueMode mode) {
        field.updateAlliance(); // running this every single loop is horrible practice but wtv

        // Update the various perception pipelines.
        updateOdometryLocalization();
        updateVisionLocalization();
        updateObjectDetection();

        // *** SMARTDASH BOARD LOGS ***
        Debug.log("Robot pitch (°)", gyro.getPitch());
        Debug.log("Robot roll (°)", gyro.getRoll());

        Debug.log("Pose", Util.pose2d2str(poseEstimator.getEstimatedPosition()));
        Debug.log("Filtered Pose", Util.pose2d2str(getFilteredPose()));
        Debug.log("Heading (°)", getHeading().getDegrees());
        Debug.log("Desired Heading Lock (°)", getHeadingLock().getDegrees());

        Debug.log("SHTR_L Status", tagCameraLeft.getStatus().toString());
        Debug.log("SHTR_R Status", tagCameraRight.getStatus().toString());
        Debug.log("INTK_R Status", intakeCamera.getStatus().toString());

        SmartDashboard.putNumber("match_time", DriverStation.getMatchTime());

        // Update the field map
        field2d.setRobotPose(getFilteredPose());
        if (!Robot.isReal() && shooter.wantsState(Shooter.State.SMART) && mode.isAuto()) {
            field2d.setRobotPose(new Pose2d(getPose().getTranslation(), getAngleToSpeaker()));
        }

        // Log the pose of the speaker using AdvantageScope
        Logger.recordOutput("Perception/SpeakerPose", new Pose2d[] {
                field.speakerPose });

        // Run rolling median filter aggregation
        filteredPose = new Pose2d(
                filteredX.calculate(getPose().getX()),
                filteredY.calculate(getPose().getY()),
                getHeading());
    }

    /**
     * Returns an "summary" of the total vision status by fusing
     * the status of both tag cameras.
     * 
     * The strategy is returning the status of the camera with
     * the most fatal status.
     */
    public Status getMostFatalVisionStatus() {
        final Status lStatus = tagCameraLeft.getStatus();
        final Status rStatus = tagCameraRight.getStatus();
        if (lStatus == Status.DOWN || rStatus == Status.DOWN) {
            return Status.DOWN;
        }
        if (lStatus == Status.STALE || rStatus == Status.STALE) {
            return Status.STALE;
        }
        return Status.OK;
    }

    /** Updates the pose estimator with swerve encoder feedback */
    public void updateOdometryLocalization() {
        poseEstimator.update(getHeading(), drivebase.getModulePositions());
    }

    // A map of all tags that are in view on this current update. Cleared between
    // updates.
    private final Map<Integer, Pose3d> tagsInView = new HashMap<>();

    private boolean seesTags = false; // do we or do we not see any apriltags

    /** Update vision pipeline */
    public void updateVisionLocalization() {
        toast.update(); // Updates all the vision pipelines.

        // This gets comented/uncomented out based on if or if not we want to use
        // vision to update our odometry while we are pathing (like physically following
        // the path)
        Debug.log("Using Vision", !drivebase.wantsState(Drivebase.State.PATHING));

        if (drivebase.wantsState(Drivebase.State.PATHING)) {
            return; // do not update vision if we are pathing
        }

        seesTags = false; // reset the seesTags field

        toast.iterCams((cam) -> {
            final var pipeOpt = cam.getPipeline(AprilTags.class);
            if (pipeOpt.isEmpty())
                return;

            // If the pipeline is down or stale then we want to ignore it.
            if (pipeOpt.get().getStatus() != Status.OK) {
                return;
            }

            final AprilTags pipe = pipeOpt.get();

            final List<AprilTagDetection> detections = pipe.getDetections();

            if (detections.size() > 0)
                seesTags = true;

            for (final AprilTagDetection detection : detections) {

                // if
                // - the detection is not "valid"
                // - the id is not an id on the field
                // - the robot is rotating too fast
                // - or the tag is too far away
                // then we ignore the detection and move on
                if (!detection.isValidDetection()
                        || !field.isIDValid(detection.id)
                        || Math.abs(gyro.getAngularVelocity().getRadians()) > MAX_ANGULAR_VELOCITY_RADS
                        || detection.getDistance2d() > MAX_DISTANCE)
                    continue;

                // get tag pose in world space
                final Pose3d tagPose = fieldMap.getTagPose(detection.id).get(); // should never fail

                // we see this tag so we add it to the tagsInView map. this is so we can log the
                // detections on advantagescope
                if (!tagsInView.containsKey(detection.id))
                    tagsInView.put(detection.id, tagPose);

                // converting from cam space to robot space by adding the camera->tag transform
                // with the center->camera transform
                Transform3d robotSpaceTransform = detection.transform.plus(cam.transform);

                robotSpaceTransform = new Transform3d(new Translation3d(
                        robotSpaceTransform.getX(),
                        -robotSpaceTransform.getY(),
                        -robotSpaceTransform.getZ()), robotSpaceTransform.getRotation());

                // converting robot space to world space using the position of the tag
                final Pose3d estPose3d = tagPose.transformBy(robotSpaceTransform);

                final Pose2d estPose = estPose3d.toPose2d();

                // if the estimated position is off the field then something is wrong and we
                // must move on
                // if (!Field.isPoseOnField(estPose))
                // continue;

                // add the processed vision messurement to the pose estimator
                poseEstimator.addVisionMeasurement(estPose, Timer.getFPGATimestamp());
            }
        });

        // Serializes and pushes the seen tags to networktables so we can view
        // detections on advantagescope
        Logger.recordOutput("Perception/TagPoses", tagsInView.values().toArray(new Pose3d[tagsInView.values().size()]));
    }

    /** Run the object detection pipeline */
    public void updateObjectDetection() {
        final Note note = getBestDetection().isPresent() ? getBestDetection().get() : Note.EMPTY;
        Debug.log("Best Detection", note.toString());
    }

    /**
     * Gyro heading (yaw, CCW around the Z-axis) as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return gyro.getHeadingCCW();
    }

    /**
     * Getter for checking if we do or do we not see any apriltags. Helpful for
     * driver feedback.
     */
    public boolean seesTags() {
        return seesTags;
    }

    /**
     * Gyro angluar velocity (yaw, CCW around the Z-axis) as a Rotation2d.
     * Represents the angle (radians or degrees) per second.
     */
    public Rotation2d getAngularVelocity() {
        return gyro.getAngularVelocity();
    }

    /** Construct and return a pose estimation using our rolling median filter */
    public Pose2d getFilteredPose() {
        return filteredPose;
    }

    private Rotation2d lastFilteredAngle; // The last filtered angle we have used

    /**
     * Calculate the desired heading lock for the drivebase auto-align during teleop
     */
    public Rotation2d getHeadingLock() {
        // If we are in auto we want to return the angle from our set
        // future shooting pose to the speaker.
        if (DriverStation.isAutonomous()) {
            return field.getAngleToSpeaker(futureShootingPose);
        }
        // If shooter is trying to laser then we want to get the angle
        // to the passing zone.
        if (shooter.wantsState(Shooter.State.LASER)) {
            return field.getAngleToPassingZone(getFilteredPose());
        }
        // If we are in smart state then we want to freeze the heading.
        // This may be removed in the future.
        if (!shooter.wantsState(Shooter.State.SMART))
            lastFilteredAngle = field.getAngleToSpeaker(getFilteredPose());
        return lastFilteredAngle;
    }

    /** Get gyro pitch, possible dead code */
    public double getGyroPitch() {
        return gyro.getPitch();
    }

    /**
     * Tare the gyro, make the current heading "north" (0° yaw) and reset the pose.
     */
    public void resetPoseAndGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        setPose(new Pose2d(0, 0, getHeading()));
    }

    /** Reset the gyro angle to 0 */
    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
    }

    /** Reset the gyro to a given angle */
    public void resetGyro(final Rotation2d offset) {
        gyro.setOffsetCW(offset);
    }

    /** Set the future pose that we use for state suspended calculations */
    public void setFutureShootingPose(final Pose2d pose) {
        futureShootingPose = pose;
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
     * Reset the position in the pose estimator to be at the origin.
     */
    public void resetPose() {
        setPose(new Pose2d());
    }

    /**
     * Reset the position in the pose estimator to be at a given position.
     */
    public void resetPose(final Pose2d pose) {
        setPose(pose);
    }

    /**
     * Get the angle from the robot to the speaker.
     */
    public Rotation2d getAngleToSpeaker() {
        return field.getAngleToSpeaker(getPose());
    }

    /** Get the distance from the robot to the speaker w/ the regular pose */
    public double getDistanceToSpeaker() {
        return field.distanceToSpeaker(getPose());
    }

    /** Get the distance from the robot to the speaker w/ the filtered pose */
    public double getFilteredDistanceToSpeaker() {
        return field.distanceToSpeaker(getFilteredPose());
    }

    /** Get the distance from the robot to the speaker */
    public double getFutureDistanceToSpeaker() {
        return field.distanceToSpeaker(futureShootingPose);
    }

    /**
     * Get target offset of the given camera, this should be [-W/2, W/2]
     * where W is the width of the frame in pixels.
     * 
     * +--------------------------+
     * | . |
     * | . |
     * | . X |
     * | . |
     * | . |
     * +--------------------------+
     * -W/2 0 W/2
     * 
     * ex. X ~= W/4
     */
    public Optional<AprilTagDetection> getTarget(final AprilTags pipeline) {
        final List<AprilTagDetection> detections = pipeline.getDetections();

        final int targetID = field.getSpeakerTargetID();

        for (final AprilTagDetection detection : detections) {

            if (!detection.isValidDetection())
                continue;

            final int id = detection.id;

            if (id != targetID)
                continue;

            return Optional.of(detection);
        }
        return Optional.empty();
    }

    final static double HALF_W = 1920.0 / 2.0;

    /**
     * Get the fused x-offset to the target tag from the shooter side of the robot.
     */
    public Optional<Double> getFusedTargetOffset() {

        // Uses shooter perspective
        final var lopt = getTarget(tagCameraLeft);
        final var ropt = getTarget(tagCameraRight);

        final double lx = lopt.isPresent() ? lopt.get().xOffset : HALF_W;
        final double rx = ropt.isPresent() ? ropt.get().xOffset : -HALF_W;

        // This is a primative algorithm that might work w/ a PID controller
        if (lopt.isPresent() && ropt.isPresent()) { // returns if in view on both
            return Optional.of(lx + rx);
        }
        return Optional.empty();
    }

    /**
     * Compute an accurate estimate of the normal distance to the target tag
     * in the XY plane.
     */
    public Optional<Double> getFusedTargetDistance() {

        // Uses shooter perspective
        final var lOpt = getTarget(tagCameraLeft);
        final var rOpt = getTarget(tagCameraRight);

        final boolean lPresent = lOpt.isPresent();
        final boolean rPresent = rOpt.isPresent();

        final Translation2d tl = lPresent ? lOpt.get().transform.getTranslation().toTranslation2d()
                : new Translation2d();
        final Translation2d tr = rPresent ? rOpt.get().transform.getTranslation().toTranslation2d()
                : new Translation2d();

        if (lPresent && rPresent) {
            final double xAvg = (tl.getX() + tr.getX()) / 2.0;
            final double yAvg = (tl.getX() + tr.getX()) / 2.0;
            return Optional.of(Math.sqrt(xAvg * xAvg + yAvg * yAvg));
        }
        if (lPresent && !rPresent) {
            return Optional.of(tl.getNorm());
        }
        if (!lPresent && rPresent) {
            return Optional.of(tr.getNorm());
        }
        return Optional.empty();
    }

    private static volatile Perception instance;

    public static synchronized final Perception getInstance() {
        return instance == null ? instance = new Perception() : instance;
    }

    /** Grab note detections from the TOAST pipeline */
    public List<Note> getNoteDetections() {
        return intakeCamera.getDetections();
    }

    /** Get the best detection of the notes. */
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

        // public static Note fromJSONLeft(final JsonNode det) {
        // final Note parsed = parseJSON(det);
        // final double angle = calculateAngle(parsed.x);
        // return new Note(parsed.name, parsed.x, angle, parsed.confidence);
        // }

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
