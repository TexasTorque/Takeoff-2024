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

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double CONTROLLER_DEADBAND = 0.025;

    private final TorqueRequestableTimeout rumbleTimeout;

    private final TorqueBoolSupplier resetGyro, speedUp, speedDown, runSmartIntake,
            runDumbIntake, runOuttake, speakerSmartShot, speakerLayup, speakerSafeZone, amp, trap, speakerWarmup,
            slowlySlowDownClick, slowlySlowDownHold, manualGateOut, manualGateIn, babyBird;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);

        rumbleTimeout = new TorqueRequestableTimeout();

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);

        speedUp = new TorqueClickSupplier(driver::isRightBumperDown);
        speedDown = new TorqueClickSupplier(driver::isLeftBumperDown);

        slowlySlowDownClick = new TorqueClickSupplier(driver::isLeftTriggerDown);
        slowlySlowDownHold = new TorqueBoolSupplier(driver::isLeftTriggerDown);

        runSmartIntake = new TorqueBoolSupplier(driver::isRightTriggerDown);
        runDumbIntake = new TorqueBoolSupplier(driver::isBButtonDown);
        runOuttake = new TorqueBoolSupplier(driver::isAButtonDown);

        speakerSmartShot = new TorqueBoolSupplier(operator::isRightTriggerDown);
        speakerWarmup = new TorqueBoolSupplier(operator::isRightBumperDown);

        speakerLayup = new TorqueBoolSupplier(operator::isYButtonDown);
        speakerSafeZone = new TorqueBoolSupplier(operator::isAButtonDown);

        amp = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        trap = new TorqueBoolSupplier(operator::isXButtonDown);

        manualGateOut = new TorqueBoolSupplier(operator::isDPADUpDown);
        manualGateIn = new TorqueBoolSupplier(operator::isDPADDownDown);

        babyBird = new TorqueBoolSupplier(operator::isLeftBumperDown);
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

        amp.onTrue(() -> shooter.setState(Shooter.State.AMP));
        trap.onTrue(() -> shooter.setState(Shooter.State.TRAP));

        speakerWarmup.onTrue(() -> shooter.setState(Shooter.State.WARMUP));

        manualGateOut.onTrue(() -> shooter.setGateState(Shooter.GateState.OUT));
        manualGateIn.onTrue(() -> shooter.setGateState(Shooter.GateState.IN));

        babyBird.onTrue(() -> shooter.setState(Shooter.State.BABYBIRD));
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetGyro());
        speedDown.onTrue(() -> drivebase.speedSetting.shiftDown());
        speedUp.onTrue(() -> drivebase.speedSetting.shiftUp());

        slowlySlowDownClick.onTrue(() -> drivebase.speedSequence = new SpeedSequence(Drivebase.SpeedSetting.FAST,
                Drivebase.SpeedSetting.SLOW, 1));

        slowlySlowDownHold.onTrue(() -> drivebase.speedSetting = SpeedSetting.SEQ);

        if (!slowlySlowDownHold.get()) drivebase.speedSetting = Drivebase.SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(-driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeedsTeleop(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
    }

    public void updateRumble() {
        driver.setRumble(rumbleTimeout.get());
        operator.setRumble(rumbleTimeout.get());
    }

    public void setRumbleFor(final double duration) {
        rumbleTimeout.set(duration);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
