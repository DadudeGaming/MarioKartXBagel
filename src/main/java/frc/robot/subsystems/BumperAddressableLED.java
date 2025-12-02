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
  AddressableLEDBufferView m_LedSection2 = m_ledBuffer.createView(74, 89);
  AddressableLEDBufferView m_LedSection3 = m_ledBuffer.createView(90, 163);
  AddressableLEDBufferView m_LedSection4 = m_ledBuffer.createView(164, 182);

  //set up patterns

  //public static LEDPattern m_visor1 = LEDPattern.
  public static LEDPattern m_off = LEDPattern.kOff;

  // one shared position and direction for both visors
private int visorPos = 0;
private int visorDir = 1;

// how many LEDs the white bar uses
private static final int kBarSize = 4;

// how often to advance the animation (smaller = faster)
private static final int kSpeed = 1;  
private int speedCounter = 0;

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
 
    //m_visor1.applyTo(m_LedSection1);
    //m_visor1.applyTo(m_LedSection3);

    m_off.applyTo(m_LedSection2);
    m_off.applyTo(m_LedSection4);
  }

  public Command setWhiteCommand() {
    return run(() -> {
      pattern1();
    });
  }

  private void drawVisor(AddressableLEDBufferView view, int startSide, int sharedPos) {
    int len = view.getLength();
  
    // Choose index mapping per section
    int localPos = (startSide == 0)
        ? sharedPos
        : (len - kBarSize - sharedPos);
  
    // Clear
    for (int i = 0; i < len; i++) {
      view.setRGB(i, 0, 0, 0);
    }
  
    // Draw bar
    for (int i = 0; i < kBarSize; i++) {
      int idx = localPos + i;
      if (idx >= 0 && idx < len) {
        view.setRGB(idx, 255, 255, 255);
      }
    }
  }

  private void updateSharedVisorState(int maxLen) {
    speedCounter++;
    if (speedCounter < kSpeed) return;
    speedCounter = 0;
  
    visorPos += visorDir;
  
    if (visorPos <= 0) {
      visorPos = 0;
      visorDir = 1;
    } else if (visorPos >= maxLen - kBarSize) {
      visorPos = Math.max(0, maxLen - kBarSize);
      visorDir = -1;
    }
  }

  @Override
  public void periodic() {

    // pick the longest section since sharedPos must fit both
    int maxLen = Math.max(m_LedSection1.getLength(),
    m_LedSection3.getLength());

    updateSharedVisorState(maxLen);

    drawVisor(m_LedSection3, 1, visorPos);  // reversed
    drawVisor(m_LedSection1, 0, visorPos);  // forward
    // This method will be called once per scheduler run.
    // Continuously send the buffer data to the LEDs.
    m_led.setData(m_ledBuffer);
  }

  
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}

