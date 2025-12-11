package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

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
  private int teleportOnce = 0;

  private int flashDelayCounter = 0;
  private static final int FLASH_DELAY = 2; // increase for slower flashes

  public int driftPos = 0;
  public int driftStage = 0; // 0 blue, 1 orange, 2 purple
  public boolean driftActive = false;

  public boolean fireActive = false;
  public double fireStartTime = 0.0; // when the glow started
  public double fireDuration = 0.0;  // how long it should last
  private static final double STAGE_DELAY = 0.5; // seconds between colors


  public enum PatternMode {
    VISOR_SWEEP,
    SWEEP_AND_FLASH,
    RED,
    DRIFT,
    OFF
  }  

  private final int[][] driftColors = {
    {0, 0, 255},     // blue
    {255, 120, 0},   // orange
    {180, 0, 255}    // purple
  };
  

  private PatternMode currentMode = PatternMode.VISOR_SWEEP;

  /** Creates a new WhiteLED subsystem. */
  public BumperAddressableLED() {
    setPatternMode(PatternMode.DRIFT); //Can be changed
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

    // clear sections 2 and 4
    for (int i = 0; i < m_LedSection2.getLength(); i++) {
      m_LedSection2.setRGB(i, 0, 0, 0);
    }
    for (int i = 0; i < m_LedSection4.getLength(); i++) {
      m_LedSection4.setRGB(i, 0, 0, 0);
    }


  }

  private void runSweepAndFlash() {
  
    // speed table slow to fast
    int[] speeds = {4, 5, 6, 7, 8};  // you can tune this
    int maxLen = Math.max(m_LedSection1.getLength(), m_LedSection3.getLength());

    if (sweepCount < 5) {

      // pick speed based on which pulse we are on
      int step = speeds[sweepCount];

      // advance forward only
      visorPos += step;

      // clear sections 1 and 3
      for (int i = 0; i < m_LedSection1.getLength(); i++) {
        m_LedSection1.setRGB(i, 0, 0, 0);
      }
      for (int i = 0; i < m_LedSection3.getLength(); i++) {
        m_LedSection3.setRGB(i, 0, 0, 0);
      }

      // turn off 2 and 4
      for (int i = 0; i < m_LedSection2.getLength(); i++) {
        m_LedSection2.setRGB(i, 0, 0, 0);
      }
      for (int i = 0; i < m_LedSection4.getLength(); i++) {
        m_LedSection4.setRGB(i, 0, 0, 0);
      }

      // draw pulse
      for (int i = 0; i < kBarSize; i++) {
        int idx = visorPos + i;

        // section 1 goes forward (normal)
        if (idx >= 0 && idx < m_LedSection1.getLength()) {
          m_LedSection1.setRGB(idx, 255, 255, 255);
        }

        // section 3 is reversed, flip the index
        if (idx >= 0 && idx < m_LedSection3.getLength()) {
          int flip = m_LedSection3.getLength() - 1 - idx;
          if (flip >= 0 && flip < m_LedSection3.getLength()) {
            m_LedSection3.setRGB(flip, 255, 255, 255);
          }
        }
      }

      // reached the end
      if (visorPos >= maxLen) {
        visorPos = 0;
        sweepCount++;
      }

      return;
    }
  
    // flash 8 times (on/off)
    if (flashCount < 16) {   // on/off pairs
      flashDelayCounter++;
      if (flashDelayCounter < FLASH_DELAY) return; // wait before toggling
      flashDelayCounter = 0;

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

      // clear sections 2 and 4
      for (int i = 0; i < m_LedSection2.getLength(); i++) {
        m_LedSection2.setRGB(i, 0, 0, 0);
      }
      for (int i = 0; i < m_LedSection4.getLength(); i++) {
        m_LedSection4.setRGB(i, 0, 0, 0);
      }

      flashCount++;
      return;
    }

    // fill ALL FOUR sections with orange
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

    teleportOnce = 1;
    sweepCount = 0;
    flashCount = 0;
    visorPos = 0;
    visorDir = 1;
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

  private void runDrift() {
    int len = m_LedSection1.getLength();
    int speed = 5; // fast fill

    double now = Timer.getFPGATimestamp();

    // only advance if enough time has passed since last stage change
    if (driftActive && (now - fireStartTime >= STAGE_DELAY || driftPos > 0)) {
        // advance
        driftPos += speed;

        // clear both rails
        for (int i = 0; i < len; i++) {
            m_LedSection1.setRGB(i, 0, 0, 0);
            m_LedSection3.setRGB(i, 0, 0, 0);
        }

        // current color
        int r = driftColors[driftStage][0];
        int g = driftColors[driftStage][1];
        int b = driftColors[driftStage][2];

        // draw fill
        for (int i = 0; i < driftPos && i < len; i++) {
            m_LedSection1.setRGB(i, r, g, b);
            int flip = len - 1 - i;
            m_LedSection3.setRGB(flip, r, g, b);
        }

        // reached end of strip
        if (driftPos >= len) {
            driftPos = 0;
            driftStage++;
            fireStartTime = now; // mark time for next stage delay
            if (driftStage >= 3) {
                driftStage = 0; // reset for next run
                driftActive = false;
            }
        }

        // clear sections 2 and 4
        for (int i = 0; i < m_LedSection2.getLength(); i++) m_LedSection2.setRGB(i, 0, 0, 0);
        for (int i = 0; i < m_LedSection4.getLength(); i++) m_LedSection4.setRGB(i, 0, 0, 0);
    }
}

  private void driftEndGlow() {
    for (int i = 0; i < m_LedSection1.getLength(); i++) m_LedSection1.setRGB(i, 255, 80, 0);
    for (int i = 0; i < m_LedSection2.getLength(); i++) m_LedSection2.setRGB(i, 255, 80, 0);
    for (int i = 0; i < m_LedSection3.getLength(); i++) m_LedSection3.setRGB(i, 255, 80, 0);
    for (int i = 0; i < m_LedSection4.getLength(); i++) m_LedSection4.setRGB(i, 255, 80, 0);
  }

  public void startDrift() {
    driftActive = true;
    driftPos = 0;
    driftStage = 0;
    fireStartTime = Timer.getFPGATimestamp(); // start timing first color
}
  

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    double maxSpeed = 0.5;
    double minSpeed = 0.2;

    // switch patterns based on speed
    if (currentMode != PatternMode.SWEEP_AND_FLASH && RobotContainer.currentSpeed >= maxSpeed) {
        // start sweep-and-flash
        setPatternMode(PatternMode.SWEEP_AND_FLASH);
        sweepCount = 0;
        flashCount = 0;
        visorPos = 0;
        visorDir = 1;
        teleportOnce = 0;
    } 
    else if (currentMode == PatternMode.SWEEP_AND_FLASH && RobotContainer.currentSpeed < minSpeed) {
        setPatternMode(PatternMode.VISOR_SWEEP);
    }

    // run current pattern
    switch (currentMode) {
        case VISOR_SWEEP:
            int maxLen = Math.max(m_LedSection1.getLength(), m_LedSection3.getLength());
            updateSharedVisorState(maxLen);
            drawVisor(m_LedSection3, 1, visorPos);
            drawVisor(m_LedSection1, 0, visorPos);
            // clear 2 & 4
            for (int i = 0; i < m_LedSection2.getLength(); i++) m_LedSection2.setRGB(i, 0,0,0);
            for (int i = 0; i < m_LedSection4.getLength(); i++) m_LedSection4.setRGB(i, 0,0,0);
            break;

        case SWEEP_AND_FLASH:
            if (teleportOnce == 0) {
              runSweepAndFlash();
            }
            break;

        case RED:
            for (int i = 0; i < m_ledBuffer.getLength(); i++) m_ledBuffer.setRGB(i, 255,0,0);
            break;

        case DRIFT:
            if (driftActive) {
                runDrift();
            } else if (fireActive) {
                double elapsed = Timer.getFPGATimestamp() - fireStartTime;
        
                if (elapsed < fireDuration) {
                    driftEndGlow();  // keep glowing fire
                } else {
                    fireActive = false;   // stop fire/glow
                    //setPatternMode(PatternMode.OFF);
                }
            }
            break;

        case OFF:
            for (int i = 0; i < m_ledBuffer.getLength(); i++) m_ledBuffer.setRGB(i, 0,0,0);
            break;
    }

    m_led.setData(m_ledBuffer);
  }
}