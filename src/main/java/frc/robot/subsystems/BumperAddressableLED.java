package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simple subsystem to control an addressable LED strip.
 */
public class BumperAddressableLED extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // PWM port for the LED strip. This must be a PWM header on the roboRIO.
  private static final int kLedPort = 9;

  // Number of LEDs on the strip.
  private static final int kLedStripLength = 60;

  /** Creates a new WhiteLED subsystem. */
  public BumperAddressableLED() {
    m_led = new AddressableLED(kLedPort);

    // Create a buffer for the LED data.
    m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data and start the LED output.
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * Sets the entire LED strip to a solid white color.
   */
  public void setWhite() {
    // Using LEDPattern is a convenient way to set all LEDs to the same color.
    LEDPattern.solid(Color.kWhite).applyTo(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    // Continuously send the buffer data to the LEDs.
    m_led.setData(m_ledBuffer);
  }
}

