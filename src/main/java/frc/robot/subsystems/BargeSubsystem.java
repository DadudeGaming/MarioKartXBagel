// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BargeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public boolean hanging = false;
  private SparkMax intakeMotor = new SparkMax(12, MotorType.kBrushless);

  public BargeSubsystem() {
      //driverController.button(3).onTrue(declare());
      intakeMotor.set(0);
  }

  

  public void setMotor(double input) {
    intakeMotor.set(input);
  }
}
