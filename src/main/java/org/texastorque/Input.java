package org.texastorque;

import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Drivebase.SpeedSequence;
import org.texastorque.subsystems.Drivebase.SpeedSetting;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double CONTROLLER_DEADBAND = 0.025;

    private final TorqueRequestableTimeout rumbleTimeout;

    private final TorqueBoolSupplier resetGyro, speedUp, speedDown, runSmartIntake,
            runDumbIntake, runOuttake, speakerSmartShot, speakerLayup, speakerSafeZone, amp, trap,
            deaccelerateClick, deaccelerateHold, manualGateOut, manualGateIn, babyBird, debugMode, speakerMid,
            shooterIdle;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);

        rumbleTimeout = new TorqueRequestableTimeout();

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);

        speedUp = new TorqueClickSupplier(() -> false);
        speedDown = new TorqueClickSupplier(() -> false);

        deaccelerateClick = new TorqueClickSupplier(driver::isLeftTriggerDown);
        deaccelerateHold = new TorqueBoolSupplier(driver::isLeftTriggerDown);

        runSmartIntake = new TorqueBoolSupplier(driver::isRightTriggerDown);
        runDumbIntake = new TorqueBoolSupplier(driver::isRightBumperDown);
        runOuttake = new TorqueBoolSupplier(driver::isLeftBumperDown);

        speakerSmartShot = new TorqueBoolSupplier(operator::isRightTriggerDown);

        speakerLayup = new TorqueBoolSupplier(operator::isYButtonDown);
        speakerSafeZone = new TorqueBoolSupplier(operator::isAButtonDown);
        speakerMid = new TorqueBoolSupplier(operator::isBButtonDown);

        amp = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        trap = new TorqueBoolSupplier(() -> false);

        manualGateOut = new TorqueBoolSupplier(operator::isDPADUpDown);
        manualGateIn = new TorqueBoolSupplier(operator::isDPADDownDown);

        shooterIdle = new TorqueToggleSupplier(operator::isRightBumperDown);

        babyBird = new TorqueBoolSupplier(operator::isLeftBumperDown);

        debugMode = new TorqueToggleSupplier(
                () -> operator.isLeftCenterButtonDown() && operator.isRightCenterButtonDown());
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateIntake();
        updateShooter();
        updateRumble();
    }

    public void updateIntake() {
        runSmartIntake.onTrue(() -> intake.setState(Intake.State.SMART_INTAKE));
        runDumbIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));
    }

    public void updateShooter() {
        speakerSmartShot.onTrue(() -> shooter.setState(Shooter.State.SMART));

        speakerLayup.onTrue(() -> shooter.setState(Shooter.State.LAYUP));
        speakerSafeZone.onTrue(() -> shooter.setState(Shooter.State.SAFEZONE));
        speakerMid.onTrue(() -> shooter.setState(Shooter.State.MID));

        amp.onTrue(() -> shooter.setState(Shooter.State.AMP));
        trap.onTrue(() -> shooter.setState(Shooter.State.TRAP));

        manualGateOut.onTrue(() -> shooter.setGateState(Shooter.GateState.OUT));
        manualGateIn.onTrue(() -> shooter.setGateState(Shooter.GateState.IN));

        babyBird.onTrue(() -> {
            shooter.setState(Shooter.State.BABYBIRD);
            shooter.setGateState(Shooter.GateState.IN);
        });

        shooter.setIdle(!shooterIdle.get());
        shooter.setConsent(!operator.isLeftStickClickDown());

        shooter.setDebugMode(debugMode.get());
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetPoseAndGyro());
        speedDown.onTrue(() -> drivebase.speedSetting.shiftDown());
        speedUp.onTrue(() -> drivebase.speedSetting.shiftUp());

        deaccelerateClick.onTrue(() -> drivebase.speedSequence = new SpeedSequence(Drivebase.SpeedSetting.FAST,
                Drivebase.SpeedSetting.SLOW, 1));

        deaccelerateHold.onTrue(() -> drivebase.speedSetting = SpeedSetting.SEQ);

        if (!deaccelerateHold.get())
            drivebase.speedSetting = Drivebase.SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeedsTeleop(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
    }

    public void updateRumble() {
        boolean rumbleLeft = rumbleTimeout.get();
        boolean rumbleRight = rumbleTimeout.get();

        if (TorqueMath.toleranced(DriverStation.getMatchTime(), 20, 1) && DriverStation.isTeleop()) {
            if (Timer.getFPGATimestamp() * 100 % 2 == 0) {
                rumbleLeft = true;
                rumbleRight = false;
            } else {
                rumbleLeft = false;
                rumbleRight = true;
            }
        }
        driver.setRumbleLeft(rumbleLeft);
        driver.setRumbleRight(rumbleRight);
        operator.setRumbleLeft(rumbleLeft);
        operator.setRumbleRight(rumbleRight);
    }

    public void setRumbleFor(final double duration) {
        rumbleTimeout.set(duration);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
