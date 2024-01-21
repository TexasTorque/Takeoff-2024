package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Camera;
import org.texastorque.toast.lib.Toast;
import org.texastorque.toast.lib.pipelines.AprilTags;
import org.texastorque.toast.lib.pipelines.AprilTags.AprilTagDetection;
import org.texastorque.toast.lib.thirdparty.BetterPoseEstimator;
import org.texastorque.toast.lib.thirdparty.BetterPoseEstimator.TimestampedVisionUpdate;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Perception extends TorqueStatorSubsystem<Perception.State> implements Subsystems {

    public enum State implements TorqueState {
        VISION;
    }

    private static final Vector<N3> INV_KINEMATICS_STDS = VecBuilder.fill(.05, .05, Units.degreesToRadians(2));
    private static final Vector<N3> VISION_STDS = VecBuilder.fill(.1, .1, Units.degreesToRadians(5));

    private static final double MAX_ANGULAR_VELOCITY = Math.PI / 2;
    private static final double MAX_DISTANCE = 4;

    private final Toast toast;

    private final SwerveDrivePoseEstimator poseEstimatorWpi;
    private final BetterPoseEstimator poseEstimatorMech;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();
    public final Field2d fieldWpi = new Field2d(), fieldMech = new Field2d();
    private AprilTagFieldLayout fieldMap = Field.getFieldLayout();

    private SwerveModulePosition[] lastModulePositions;
    private Rotation2d lastHeading;

    public Perception() {
        super(State.VISION);

        toast = new Toast();
        poseEstimatorWpi = new SwerveDrivePoseEstimator(
                drivebase.kinematics,
                getHeading(),
                drivebase.getModulePositions(),
                new Pose2d(), INV_KINEMATICS_STDS, VISION_STDS);

        poseEstimatorMech = new BetterPoseEstimator(INV_KINEMATICS_STDS);

        final double a = 8.5;
        final double b = 10.52;
        final double h = 9.446;
        final double p = 51;
        // final double p = 0;

        toast.addCamera(new Camera("sim", Camera.transformInchDeg(b, a, h, 0, p, 0)));

        // toast.addCamera(new Camera("fl", Camera.transformInchDeg(b, a, h, 0, p, 0)));
        // toast.addCamera(new Camera("fr", Camera.transformInchDeg(b, -a, h, 0, p,
        // 0)));

        // toast.addCamera(new Camera("ll", Camera.transformInchDeg(-a, b, h, 0, p,
        // 90)));
        // toast.addCamera(new Camera("lr", Camera.transformInchDeg(a, b, h, 0, p,
        // 90)));

        // toast.addCamera(new Camera("bl", Camera.transformInchDeg(-b, a, h, 0, p,
        // 180)));
        // toast.addCamera(new Camera("br", Camera.transformInchDeg(-b, -a, h, 0, p,
        // 180)));

        // toast.addCamera(new Camera("rl", Camera.transformInchDeg(a, -b, h, 0, p,
        // 270)));
        // toast.addCamera(new Camera("rr", Camera.transformInchDeg(-a, -b, h, 0, p,
        // 270)));

        toast.iterCams(cam -> cam.addPipeline(new AprilTags(cam.id)));

        Debug.field("Field (WPI)", fieldWpi);
        Debug.field("Field (Mech)", fieldMech);

        lastModulePositions = drivebase.getModulePositions();
        lastHeading = getHeading();
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    public double getDistanceToTarget() {
        return poseEstimatorWpi.getEstimatedPosition().getTranslation().getNorm();
    }

    @Override
    public void update(TorqueMode mode) {
        updateInvKinematics();
        updateVision();

        fieldWpi.setRobotPose(poseEstimatorWpi.getEstimatedPosition());
        fieldMech.setRobotPose(poseEstimatorMech.getLatestPose());

        Debug.log("Pose (WPI)", pose2d2str(poseEstimatorWpi.getEstimatedPosition()));
        Debug.log("Pose (Mech)", pose2d2str(poseEstimatorMech.getLatestPose()));
        Debug.log("Heading (°)", getHeading().getDegrees());
    }

    public double compressedTimestamp() {
        return (((double) System.currentTimeMillis()) / 1000d) % 10000d;
    }

    public double compressedTimestamp(final double seconds) {
        return seconds % 10000d;
    }

    public void updateInvKinematics() {
        poseEstimatorWpi.update(getHeading(), drivebase.getModulePositions());

        // Calculate delta positions
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4], modules = drivebase.getModulePositions();
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(modules[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    modules[i].angle);
            lastModulePositions[i] = modules[i];
        }

        // Calculate twist
        var twist = drivebase.kinematics.toTwist2d(wheelDeltas);
        var heading = getHeading();
        twist = new Twist2d(twist.dx, twist.dy, heading.minus(lastHeading).getRadians());
        lastHeading = heading;

        // Send drivebase data to poseEstimator
        poseEstimatorMech.addDriveData(compressedTimestamp(), twist);
    }

    private final List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    public void updateVision() {
        toast.update();

        toast.iterCams((cam) -> {
            final var pipeOpt = cam.getPipeline(AprilTags.class);
            if (pipeOpt.isEmpty())
                return;
            final AprilTags pipe = pipeOpt.get();

            final List<AprilTagDetection> detections = pipe.getDetections();

            visionUpdates.clear();

            for (final AprilTagDetection detection : detections) {
                if (!detection.isValidDetection()
                        || !Field.isIDValid(detection.id)
                        || Math.abs(gyro.getAngularVelocity().getRadians()) > MAX_ANGULAR_VELOCITY
                        || detection.getDistance() > MAX_DISTANCE)
                    continue;

                final var tagPose = fieldMap.getTagPose(detection.id).get(); // should never fail

                final var camPose = tagPose.transformBy(detection.transform.inverse());

                final var estPose = camPose.transformBy(cam.transform).toPose2d();

                if (!Field.isPoseOnField(estPose))
                    continue;

                poseEstimatorWpi.addVisionMeasurement(estPose, detection.timestamp);

                visionUpdates.add(
                        new TimestampedVisionUpdate(compressedTimestamp(detection.timestamp), estPose, VISION_STDS));
            }
            poseEstimatorMech.addVisionData(visionUpdates);
        });
    }

    public Rotation2d getHeading() {
        return gyro.getHeadingCCW();
    }

    public Rotation2d getAngularVelocity() {
        return gyro.getAngularVelocity();
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
    }

    public void resetPose() {
        poseEstimatorWpi.resetPosition(getHeading(), drivebase.getModulePositions(), new Pose2d());
        poseEstimatorMech.resetPose(new Pose2d());
    }

    private String pose2d2str(final Pose2d pose) {
        return String.format("(%.2fm, %.2fm) @ %.2f°", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return poseEstimatorWpi.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimatorWpi.resetPosition(getHeading(), drivebase.getModulePositions(), pose);
    }

    public void setGoalPose(Pose2d pose) {
        fieldWpi.getObject("goalPose").setPose(pose);
    }

    private static volatile Perception instance;

    public static synchronized final Perception getInstance() {
        return instance == null ? instance = new Perception() : instance;
    }

}
