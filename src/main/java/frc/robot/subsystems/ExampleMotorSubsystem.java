// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleMotorSubsystem extends SubsystemBase {

  // declare variables up here, you can either set their value here, or later in the code, it mostly just depends on what you're doing

  // initialize a variable for keeping track of the intake state
  private boolean intakeMode; 

  // create a new spark max (neo motor controller) called motor1, at CAN ID 12, and tell it that it is a brushless motor, since spark max
  private SparkMax motor1 = new SparkMax(12, MotorType.kBrushless);

  /** Creates a new ExampleMotorSubsystem. */
  public ExampleMotorSubsystem() {
    intakeMode = false; // default to ourtake first (we have a preloaded game piece)
  }



  // Commands can either run
  public Command runMotors() {
    // depending on what the intake bool is, return either the intake or outake command.
    // when we actually make the robot, we might want to use a sensor to detect if there's a coral in the robot already instead of just toggling
    if(intakeMode){
      return intake().withTimeout(3); // return the correct command to run, with a 3 second timeout, so it stops after 3 seconds
    } else {
      return outake().withTimeout(3);
    }
  }


  public Command intake(){

    // a runEnd command has two parts, the first part is what it is running while it is being scheduled, 
    // then the second part is what is called when it ends (in this case by timing out), eg. stopping the motors so they don't continue running
    return runEnd(
      () -> {
        motor1.set(1); // set the motor to 100% speed in
      }, () -> {
        intakeMode = false; // set the intake mode to false (we need to outake next)
        motor1.set(0); // stop the motor, otherewise it would continue running forever
      }
    );
  }

  public Command outake(){
    return runEnd(
      () -> {
        motor1.set(-1); // set the motor to 100% speed out
      }, () -> {
        intakeMode = true; // set the intake mode to false (we need to intake next)
        motor1.set(0);// stop the motor, otherewise it would continue running forever
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, constantly, usually you'll put logging stuff here
  }
}
