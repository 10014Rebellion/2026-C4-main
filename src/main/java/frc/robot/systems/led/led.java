package frc.robot.systems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;

// NO IO LAYER!!!
public class led {
    public static boolean enabled = false;
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public led(int numberOfLEDs, int port) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(numberOfLEDs);
        leds.setData(buffer);
    }

    public void startAnimating() {
        leds.start();
    }

    public void setSolid(Color color) {
        LEDPattern solidPattern = LEDPattern.solid(color);
        solidPattern.applyTo(buffer);
        leds.setData(buffer); // Reapply the data in case you need to.
    }
}
