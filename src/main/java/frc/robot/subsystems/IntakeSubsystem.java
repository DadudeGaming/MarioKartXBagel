// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public boolean holding = false;
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.MotorCANID, MotorType.kBrushless);

  private Canandcolor canandcolor = new Canandcolor(IntakeConstants.ColourCANID);

  public IntakeSubsystem() {
      //driverController.button(3).onTrue(declare());
      intakeMotor.set(0);
  }

  public double getDist(){
    return canandcolor.getProximity();
  }

  public void setMotor(double input) {
    intakeMotor.set(input);
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
