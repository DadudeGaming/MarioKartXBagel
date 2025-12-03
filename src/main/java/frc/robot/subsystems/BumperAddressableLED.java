package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
//import edu.wpi.first.wpilibj.util.Color;
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
  private static final int kBarSize = 8;
  private static final int kStep = 8;   // how many LEDs it moves per tick

  private int sweepCount = 0;
  private int flashCount = 0;
  private boolean flashOn = false;
  private long waitUntil = 0;


  public enum PatternMode {
    VISOR_SWEEP,
    SWEEP_AND_FLASH
  }  

  private PatternMode currentMode = PatternMode.VISOR_SWEEP;

  /** Creates a new WhiteLED subsystem. */
  public BumperAddressableLED() {
    setPatternMode(PatternMode.SWEEP_AND_FLASH); //Can be changed
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

  public void setPatternMode(PatternMode mode) {
    currentMode = mode;
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

  private void runSweepAndFlash() {
    int maxLen = Math.max(m_LedSection1.getLength(), m_LedSection3.getLength());
  
    // playing the 5 sweeps
    if (sweepCount < 5) {
      updateSharedVisorState(maxLen);
      drawVisor(m_LedSection3, 1, visorPos);
      drawVisor(m_LedSection1, 0, visorPos);
  
      if (visorPos == 0 || visorPos == maxLen - kBarSize) {
        sweepCount++;
      }
      return;
    }
  
    // flash 8 times (on/off)
    if (flashCount < 16) {   // on/off pairs
      flashOn = !flashOn;
  
      int r = flashOn ? 255 : 0;
      int g = flashOn ? 255 : 0;
      int b = flashOn ? 255 : 0;
  
      for (int i = 0; i < m_LedSection1.getLength(); i++) {
        m_LedSection1.setRGB(i, r, g, b);
      }
      for (int i = 0; i < m_LedSection3.getLength(); i++) {
        m_LedSection3.setRGB(i, r, g, b);
      }
  
      flashCount++;
      return;
    }
  
    // reset and wait 5 seconds
    if (waitUntil == 0) {
      waitUntil = System.currentTimeMillis() + 5000;
      return;
    }

    // fill ALL FOUR sections with orange while waiting
    for (int i = 0; i < m_LedSection1.getLength(); i++) {
      m_LedSection1.setRGB(i, 255, 80, 0);
    }
    for (int i = 0; i < m_LedSection2.getLength(); i++) {
      m_LedSection2.setRGB(i, 255, 80, 0);
    }
    for (int i = 0; i < m_LedSection3.getLength(); i++) {
      m_LedSection3.setRGB(i, 255, 80, 0);
    }
    for (int i = 0; i < m_LedSection4.getLength(); i++) {
      m_LedSection4.setRGB(i, 255, 80, 0);
    }
  
    if (System.currentTimeMillis() >= waitUntil) {
      sweepCount = 0;
      flashCount = 0;
      waitUntil = 0;
      visorPos = 0;
      visorDir = 1;
    }
  }
  


  private void updateSharedVisorState(int maxLen) {
    visorPos += visorDir * kStep;
  
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

  switch (currentMode) {

    case VISOR_SWEEP:
      int maxLen = Math.max(m_LedSection1.getLength(), m_LedSection3.getLength());
      updateSharedVisorState(maxLen);
      drawVisor(m_LedSection3, 1, visorPos);
      drawVisor(m_LedSection1, 0, visorPos);
      break;

    case SWEEP_AND_FLASH:
      runSweepAndFlash();
      break;
  }

  for (int i = 0; i < m_LedSection2.getLength(); i++) {
    m_LedSection2.setRGB(i, 0, 0, 0);
  }
  for (int i = 0; i < m_LedSection4.getLength(); i++) {
    m_LedSection4.setRGB(i, 0, 0, 0);
  }

  m_led.setData(m_ledBuffer);
}


  
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}

