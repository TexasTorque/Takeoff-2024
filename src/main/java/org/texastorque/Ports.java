package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
    public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 11);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);

    public static final int SHOOTER_ROTARY = 13;
    public static final int SHOOTER_ROTARY_ENCODER = 15;
    public static final int FLYWHEEL_TOP = 16;
    public static final int FLYWHEEL_TOP_ENCODER = 17;
    public static final int FLYWHEEL_BOTTOM = 18;
    public static final int FLYWHEEL_BOTTOM_ENCODER = 19;
    public static final int SHOOTER_GATE = 20;

    public static final int INTAKE_ROTARY_LEFT = 22;
    public static final int INTAKE_ROTARY_RIGHT = 23;
    public static final int INTAKE_ROLLERS = 24;

    // TBD
    public static final int CLIMBER_LEFT = 25;
    public static final int CLIMBER_RIGHT = 26;
    public static final int CHUTE = 21;
    public static final int CHUTE_ENCODER = 14;

    public static final int LIGHTS_SUPERSTRUCTURE = 0;
    public static final int LIGHTS_CLIMBER_LEFT = 1;
    public static final int LIGHTS_CLIMBER_RIGHT = 2;

    public static final int SHOOTER_NOTE_SENSOR = 0;
}
