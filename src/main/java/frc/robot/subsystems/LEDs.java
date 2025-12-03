package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    // all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);

    public LEDs() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(183);
        m_led.setLength(m_ledBuffer.getLength());

        // m_rainbow.applyTo(m_ledBuffer);
        funnyPattern();

        // Set the data
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    public void funnyPattern() {
        int length = m_ledBuffer.getLength();
        boolean goldGroup = true;
        boolean greenGroup = false;
        int ledGroupCounter = 0;
        for (int i = 0; i < length; i++) {
            // if (i > 0) {
            // m_ledBuffer.setLED(i - 1, Color.kBlack);
            // }

            if (greenGroup) {
                if (ledGroupCounter <= 4) {
                    m_ledBuffer.setLED(i, Color.kDarkGreen);
                    ledGroupCounter++;
                } else {
                    greenGroup = false;
                    goldGroup = true;
                    ledGroupCounter = 0;
                }
            }

            if (goldGroup) {
                if (ledGroupCounter <= 4) {
                    m_ledBuffer.setLED(i, new Color(255, 165, 0));
                    ledGroupCounter++;
                } else {
                    goldGroup = false;
                    greenGroup = true;
                    ledGroupCounter = 0;
                }
            }
        }
    }

    @Override
    public void periodic() {

    }
}
