// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class BargeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public boolean hanging = false; 
  public double angle = 0.0;
  private double output = 0.0;
  private SparkMax bargeMotor = new SparkMax(12, MotorType.kBrushless);

  private final PIDController forwardController = new PIDController(ClimbConstants.PIDConstants.kP, ClimbConstants.PIDConstants.kI, ClimbConstants.PIDConstants.kD);
  private Canandmag canandmag = new Canandmag(ClimbConstants.encoderId);

  public BargeSubsystem() {
    //driverController.button(3).onTrue(declare());
    bargeMotor.set(0);
  }

  public double encoderInDegrees(){
    return canandmag.getAbsPosition() * 360;
  }

  public void runPID() {
      output = forwardController.calculate(encoderInDegrees());

      bargeMotor.set(output);
  }
  public void setClimbAngle(double input) {
    forwardController.setSetpoint(input);
 }

  public void setMotor(double input) {
    bargeMotor.set(input);
  }
  
  @Override
  public void periodic()
  {
    runPID();
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
