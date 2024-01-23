package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
    public static final SwervePorts FL_MOD = new SwervePorts(2, 1, 11);
    public static final SwervePorts FR_MOD = new SwervePorts(4, 3, 9);
    public static final SwervePorts BR_MOD = new SwervePorts(6, 5, 10);
    public static final SwervePorts BL_MOD = new SwervePorts(8, 7, 12);

    public static final int FLYWHEEL_LEFT = 13;
    public static final int FLYWHEEL_RIGHT = 14;
    public static final int SHOOTER_ROTARY = 15;
    public static final int SHOOTER_ROTARY_ENCODER = 16;
    public static final int SHOOTER_GATE = 25;

    public static final int INTAKE_ROTARY_LEFT = 16;
    public static final int INTAKE_ROTARY_RIGHT = 17;
    public static final int INTAKE_ROLLERS = 27;

    public static final int CLIMBER_LEFT = 18;
    public static final int CLIMBER_RIGHT = 19;
    public static final int HOOK = 20;

    public static final int LIGHTS_SUPERSTRUCTURE = 0;
}
