/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Bravo/Charlie/Takeoff-2024, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Pipeline.Status;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueStatelessSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LED light controller. Basically a state observer.
 */
public final class Lights extends TorqueStatelessSubsystem implements Subsystems {

    /** LightAction to set the LED to a constant color */
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

    /** LightAction to set the LED to blink a set color at a set hertz */
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

    /** LightAction to set the LED to display a rainbow */
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

    private final List<AddressableLED> lights;
    private final AddressableLEDBuffer buff;

    // Leave all these combinations
    private LightAction rainbow = new Rainbow(), 

        red = new Solid(() -> Color.kRed), 
        blinkRed = new Blink(() -> Color.kRed, 6),
        
        yellow = new Solid(() -> Color.kYellow),
        blinkYellow = new Blink(() -> Color.kYellow, 6),

        green = new Solid(() -> Color.kGreen),
        blinkGreen = new Blink(() -> Color.kGreen, 6),

        blue = new Solid(() -> Color.kBlue),
        blinkBlue = new Blink(() -> Color.kBlue, 6),

        purple = new Solid(() -> Color.kPurple),
        blinkPurple = new Blink(() -> Color.kPurple, 6);

    private Lights() {
        lights = new ArrayList<>();
        buff = new AddressableLEDBuffer(LENGTH);

        createStrips(
                Ports.LIGHTS_SUPERSTRUCTURE);
    }

    /** Instantiate the various color strips */
    private void createStrips(int... ports) {
        for (int i = 0; i < buff.getLength(); i++) {
            buff.setLED(i, Color.kGreen);
        }

        for (int port : ports) {
            final AddressableLED strip = new AddressableLED(port);
            strip.setLength(LENGTH);
            lights.add(strip);
        }

        setData(buff);
    }

    /** Set the LED buffer to the lights */
    private void setData(final AddressableLEDBuffer buff) {
        for (final AddressableLED strip : lights) {
            strip.setData(buff);
        }
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        for (final AddressableLED strip : lights) {
            strip.start(); // start all LED strips
        }
    }

    public final LightAction getColor(final TorqueMode mode) {
        // Not prsent: first we check if we are in debug mode and blink yellow

        // We must check the vision status of vision and return the 
        // failure conditions if necessary
        final Status visionStatus = perception.getMostFatalVisionStatus();
        if (visionStatus == Status.STALE) {
            return blinkYellow;
        } 
        if (visionStatus == Status.DOWN) {
            return blinkRed;
        }


        // We go rainbow if we are in climb mode
        if (Input.getInstance().isClimbing()) {
            return rainbow;
        }

        // Otherwise we check if we have a note
        if (shooter.hasNote()) {
            // And then if we see a tag
            if (perception.seesTags()) {
                // If we do see a tag we want to blink...
                // ...either green for normal mode and purple for shift mode
                return shooter.isShift() ? blinkPurple : blinkGreen;
            } else {
                // And if we do not see a tag we want to be solid...
                // ...again, either green for normal mode and purple for shift mode
                return shooter.isShift() ? purple : green;
            }
            // And if we dont have a note then we want to be solid red
        } else {
            return red;
        }

    }

    @Override
    public final void update(final TorqueMode mode) {
        // We need to get and process the color, then set that buffer to the LED strips
        getColor(mode).run(buff);
        setData(buff);
    }

    @Override
    public void clean(TorqueMode mode) {
    }
}