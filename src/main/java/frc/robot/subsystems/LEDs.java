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

    int timer = 51;
    boolean goldGroup = true;
    boolean greenGroup = false;

    private int m_fillRadius = 0; // distance expanded from the center
    private int m_fillCenter = 0; // starting point
    private int m_fillSpeedCounter = 0;
    private int m_fillSpeed = 1; // calls per radius step (higher = slower)

    boolean doneFilling = false;
    int currentColour = 1;

    public Color blue = new Color(0, 0, 255);
    public Color orange = new Color(255, 120, 0);
    public Color purple = new Color(180, 0, 255);
    public Color[] driftColors = { Color.kBlack, blue, orange, purple };

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
    }

    public void funnyPattern() {
        int length = m_ledBuffer.getLength();
        int ledGroupCounter = 0;

        if (timer > 50) {
            timer = 0;
        } else {
            timer++;
            return;
        }

        for (int i = 0; i < length; i++) {

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

            if (ledGroupCounter == 4) {
                ledGroupCounter = 0;
                goldGroup = !goldGroup;
                greenGroup = !greenGroup;
            }
        }
    }

    public void driftPattern(Color fillColor, int centerIndex, boolean reset, Color background) {
        int length = m_ledBuffer.getLength();
        centerIndex = ((centerIndex % length) + length) % length; // safe wrap

        if (reset || centerIndex != m_fillCenter) {
            m_fillCenter = centerIndex;
            m_fillRadius = 0;
            m_fillSpeedCounter = 0;
            // doneFilling = true;
            // currentColour++;
        }

        // advance radius based on speed
        m_fillSpeedCounter++;
        if (m_fillSpeedCounter >= m_fillSpeed) {
            m_fillSpeedCounter = 0;
            if (m_fillRadius < length / 2 + (length % 2)) { // enough to cover entire ring
                m_fillRadius++;
            }
        }

        // draw
        for (int i = 0; i < length; i++) {
            int d = Math.abs(i - m_fillCenter);
            d = Math.min(d, length - d); // circular distance

            if (d <= m_fillRadius) {
                m_ledBuffer.setLED(i, fillColor);
            } else {
                m_ledBuffer.setLED(i, background);
            }
        }
    }

    int driftTimer = 0;

    @Override
    public void periodic() {
        funnyPattern();
        // driftPattern(driftColors[currentColour], 0, false, driftColors[currentColour - 1]);

        // if (driftTimer >= 150) {
        //     if (!(currentColour >= driftColors.length - 1)) {
        //         currentColour++;
        //         driftTimer = 0;
        //         driftPattern(driftColors[currentColour], 0, true, driftColors[currentColour]);
        //     }
        // }
        // driftTimer++;

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}
