package org.texastorque;

import org.texastorque.subsystems.*;

public interface Subsystems {
    public final Field field = Field.getInstance();
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Perception perception = Perception.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Lights lights = Lights.getInstance();
    // public final Climber climber = Climber.getInstance();
}
