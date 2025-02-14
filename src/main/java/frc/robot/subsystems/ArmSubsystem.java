// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public double angle;

  public boolean up = false;
  public boolean down = false;

  private final CommandPS5Controller driverController = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

  // Please read here:
  /* This is simply a digital subsystem, it's not intended for actual usage, only for simulatating the arm.*/

  public ArmSubsystem() {
    angle = 0.0;
  }

  public void changeAngle(double input) {
    angle += input*0.01;
  }

  // public Command declare() {
  //   //Flip the holding variable
  //   if(holding)  //If we are holding we know the button press is intended to drop the coral
  //     return IntakeCommand.execute(-1).withTimeout(intakeTime);
  //   else //if we are not holding we know its intended to recieve coral
  //     return IntakeCommand.turn(1).withTimeout(intakeTime); //commands ran in command folder as independent script
  // }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  /*public Command take() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
      () -> {
        motor.set(1);
      }, () -> {
        holding = true;
        motor.set(0);
      }
    );
  }*/

  /*public Command turn(int speed) {
    return runOnce(
      () -> {
        motor.set(speed);
      }, () -> {
        if()
        {
          holding = !holding;
          motor.set(0);
        }
      }
    );
  }*/
}
