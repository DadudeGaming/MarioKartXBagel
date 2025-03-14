// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class ArduinoLEDSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SerialPort arduino;

  

  public ArduinoLEDSubsystem() {
    try {
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
    }
    catch(Exception e) {
      try {
        arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
      }
      catch(Exception e1) {
        try {
          arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
        }
        catch(Exception e2) {
  
        }
      }
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  /*public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here 
        });
  */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  public void SendLEDInput(byte input)
  {
    arduino.write(new byte[] {input}, 1);
    CheckInputRecieved(input);
  }

  public void CheckInputRecieved(byte input)
  {
    if(arduino.getBytesReceived() == 0)
      SendLEDInput(input);
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if(DriverStation.isEStopped())
      SendLEDInput((byte)4); //Flashing red
    else if(DriverStation.isDisabled())
      SendLEDInput((byte)0); //Slowly fading between Green and Yellow, or Green, White, and Yellow?
    else if(DriverStation.isAutonomousEnabled())
      SendLEDInput((byte)2); //Flashing Yellow and Green (High Speed)
    else if(ClimbSubsystem.angle >= 60.0)
      SendLEDInput((byte)5); //Green
    else if(Timer.getMatchTime() <= 20.0)
      SendLEDInput((byte)3); //Flashing Yellow (Hold yellow if possible then flick off then back on quickly)
    else if(DriverStation.isTeleopEnabled())
      SendLEDInput((byte)1); //Flashing Yellow and Green (Medium Speed)
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
