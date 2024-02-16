/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.function.Supplier;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueStatelessSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueStatelessSubsystem implements Subsystems {
    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, color.get());
        }
    }

    public static class Blink extends LightAction {
        private final Supplier<Color> color1, color2;
        private final double hertz;

        public Blink(final Supplier<Color> color1, final double hertz) {
            this(color1, () -> Color.kBlack, hertz);
        }

        public Blink(final Supplier<Color> color1, final Supplier<Color> color2, final double hertz) {
            this.color1 = color1;
            this.color2 = color2;
            this.hertz = hertz;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * hertz) % 2 == 1);
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, on ? color1.get() : color2.get());
        }
    }

    public static class Rainbow extends LightAction {
        private int rainbowFirstPixelHue = 0;

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (var i = 0; i < buff.getLength(); i++) {
                final int hue = (rainbowFirstPixelHue + (i * 180 / buff.getLength())) % 180;
                buff.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
        }
    }

    private static abstract class LightAction {
        public abstract void run(AddressableLEDBuffer buff);
    }

    private static volatile Lights instance;

    private static final int LENGTH = 300;

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }

    private final AddressableLED shooterLEDs;

    private final AddressableLEDBuffer buff;

    private LightAction blinkGreen = new Blink(() -> Color.kGreen, 6),
            green = new Solid(() -> Color.kGreen),
            blinkYellow = new Blink(() -> Color.kYellow, 6),
            rainbow = new Rainbow(), blue = new Solid(() -> Color.kBlue), red = new Solid(() -> Color.kRed);

    private Lights() {
        shooterLEDs = new AddressableLED(Ports.LIGHTS_SUPERSTRUCTURE);
        shooterLEDs.setLength(LENGTH);

        buff = new AddressableLEDBuffer(LENGTH);

        for (int i = 0; i < buff.getLength(); i++)
            buff.setLED(i, Color.kGreen);

        shooterLEDs.setData(buff);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        shooterLEDs.start();
    }

    public final LightAction getColor(final TorqueMode mode) {
        if (shooter.wantsToShoot())
            return blinkGreen;
        if (shooter.hasNote())
            return green;
        if (intake.isIntaking())
            return blinkYellow;
        if (mode.isAuto())
            return rainbow;

        // Teleop:
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? blue : red;
        }
        return blue;

    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor(mode).run(buff);
        shooterLEDs.setData(buff);
    }

    @Override
    public void clean(TorqueMode mode) {
    }
}