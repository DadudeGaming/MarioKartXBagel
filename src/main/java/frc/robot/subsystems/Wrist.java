// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.CANIDs;

public class Wrist extends SubsystemBase {

  private final SparkMax zMotor = new SparkMax(CANIDs.zMotor, MotorType.kBrushless);
  private final SparkMax yMotor = new SparkMax(CANIDs.yMotor, MotorType.kBrushless);
  private final SparkMax xMotor = new SparkMax(CANIDs.xMotor, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public Wrist() {}

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setAxes (double xIn, double yIn){
    double out1 = xIn - yIn;
    double out2 = xIn + yIn;

    xMotor.set(out1);
    yMotor.set(out2);
  }
}
