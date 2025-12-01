package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simple subsystem to control an addressable LED strip.
 */
public class BumperAddressableLED extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);;

  // PWM port for the LED strip. This must be a PWM header on the roboRIO.
  private static final int kLedPort = 0;

  // Number of LEDs on the strip.
  private static final int kLedStripLength = 183;

  AddressableLEDBufferView m_LedSection1 = m_ledBuffer.createView(0, 73);
  AddressableLEDBufferView m_LedSection2 = m_ledBuffer.createView(74, 90);
  AddressableLEDBufferView m_LedSection3 = m_ledBuffer.createView(91, 163);
  AddressableLEDBufferView m_LedSection4 = m_ledBuffer.createView(164, 182);

  /** Creates a new WhiteLED subsystem. */
  public BumperAddressableLED() {
    m_led = new AddressableLED(kLedPort);

    // Create a buffer for the LED data.

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data and start the LED output.
    m_led.setData(m_ledBuffer);
    m_led.start();

   

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  /**
   * Sets the entire LED strip to a solid white color.
   */
  public void pattern1() {
    // Using LEDPattern is a convenient way to set all LEDs to the same color.
    LEDPattern.solid(Color.kWhite).applyTo(m_LedSection1);
    LEDPattern.solid(Color.kRed).applyTo(m_LedSection2);
    LEDPattern.solid(Color.kWhite).applyTo(m_LedSection3);
    LEDPattern.solid(Color.kRed).applyTo(m_LedSection4);
  }

  public Command setWhiteCommand() {
    return run(() -> {
      pattern1();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    // Continuously send the buffer data to the LEDs.
    m_led.setData(m_ledBuffer);
  }

  
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}

