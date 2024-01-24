package org.texastorque;

import org.texastorque.subsystems.*;
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

    private final static double CONTROLLER_DEADBAND = 0.025; // this should be pretty small

    private final TorqueRequestableTimeout rumbleTimeout;

    private final TorqueBoolSupplier resetGyro, speedUp, speedDown, climbUp, climbDown, hookOut, hookIn, runSmartIntake,
            runDumbIntake,
            speakerSmartShot, speakerLayup, speakerSafeZone, amp, trap, speakerWarmup, ampWarmup;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);
        rumbleTimeout = new TorqueRequestableTimeout();

        resetGyro = new TorqueBoolSupplier(driver::isRightCenterButtonDown);
        speedUp = new TorqueClickSupplier(driver::isRightBumperDown);
        speedDown = new TorqueClickSupplier(driver::isLeftBumperDown);

        runSmartIntake = new TorqueBoolSupplier(driver::isRightTriggerDown);
        runDumbIntake = new TorqueBoolSupplier(driver::isRightBumperDown);

        speakerSmartShot = new TorqueBoolSupplier(operator::isRightTriggerDown);
        speakerWarmup = new TorqueToggleSupplier(operator::isRightBumperDown);
        speakerLayup = new TorqueBoolSupplier(operator::isYButtonDown);
        speakerSafeZone = new TorqueBoolSupplier(operator::isAButtonDown);
        amp = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        ampWarmup = new TorqueToggleSupplier(operator::isLeftBumperDown);
        trap = new TorqueBoolSupplier(operator::isXButtonDown);

        climbUp = new TorqueBoolSupplier(driver::isDPADUpDown);
        climbDown = new TorqueBoolSupplier(driver::isDPADDownDown);
        hookOut = new TorqueBoolSupplier(driver::isRightStickClickPressed);
        hookIn = new TorqueBoolSupplier(driver::isLeftStickClickPressed);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateClimber();
        updateIntake();
        updateShooter();
        updateRumble();
    }

    public void updateClimber() {
        climbUp.onTrue(() -> climber.setState(Climber.State.UP));
        climbDown.onTrue(() -> climber.setState(Climber.State.DOWN));
        hookOut.onTrue(() -> climber.setHookState(Climber.HookState.OUT));
        hookIn.onTrue(() -> climber.setHookState(Climber.HookState.IN));
    }

    public void updateIntake() {
        runSmartIntake.onTrue(() -> intake.setState(Intake.State.SMART_INTAKE));
        runDumbIntake.onTrue(() -> intake.setState(Intake.State.SMART_INTAKE));
    }

    public void updateShooter() {
        runSmartIntake.onTrue(() -> shooter.setState(Shooter.State.INTAKE));
        speakerSmartShot.onTrue(() -> shooter.setState(Shooter.State.SMART));

        speakerLayup.onTrue(() -> {
            shooter.setState(Shooter.State.LAYUP);
        });
        speakerSafeZone.onTrue(() -> {
            shooter.setState(Shooter.State.SAFEZONE);
        });

        amp.onTrue(() -> shooter.setState(Shooter.State.AMP));
        trap.onTrue(() -> shooter.setState(Shooter.State.TRAP));

        speakerWarmup.onTrue(() -> {
            shooter.setState(Shooter.State.WARMUP);
        });

        ampWarmup.onTrue(() -> {
            shooter.setState(Shooter.State.WARMUP);
        });
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> perception.resetGyro());
        speedDown.onTrue(() -> drivebase.getState().shiftDown());
        speedUp.onTrue(() -> drivebase.getState().shiftUp());

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(driver.getRightXAxis(), CONTROLLER_DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeeds(new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity));
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
