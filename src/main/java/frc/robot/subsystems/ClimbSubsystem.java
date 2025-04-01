// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.event.PrintJobListener;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ClimbSubsystem extends SubsystemBase {

  // declare variables up here, you can either set their value here, or later in the code, it mostly just depends on what you're doing

  public double angle;

  // create a new spark max (neo motor controller) called motor1, at CAN ID 12, and tell it that it is a brushless motor, since spark max
  private SparkMax climbMotor = new SparkMax(11, MotorType.kBrushless);

  /** Creates a new ExampleMotorSubsystem. */
  public ClimbSubsystem() { // default to ourtake first (we have a preloaded game piece)
    angle = climbMotor.getEncoder().getPosition();
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Climb", () -> climbMotor.getEncoder().getPosition());
  }

  public double getEncoder(){
    return climbMotor.getEncoder().getPosition();
  }


  public Command intake(){

    // a runEnd command has two parts, the first part is what it is running while it is being scheduled, 
    // then the second part is what is called when it ends (in this case by timing out), eg. stopping the motors so they don't continue running
    return runEnd(
      () -> {
        if(climbMotor.getEncoder().getPosition() / 100 < 55){
          // climbMotor.set(0.5);
        }
         // set the motor to 100% speed in
      }, () -> {// set the intake mode to false (we need to outake next)
        climbMotor.set(0); // stop the motor, otherewise it would continue running forever
      }
    );
  }

  public Command outake(){
    return runEnd(
      () -> {
        // climbMotor.set(-0.5); // set the motor to 100% speed out
      }, () -> { // set the intake mode to false (we need to intake next)
        climbMotor.set(0);// stop the motor, otherewise it would continue running forever
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, constantly, usually you'll put logging stuff here
  }

  public double getPos(){
    return climbMotor.getEncoder().getPosition();
  }
}