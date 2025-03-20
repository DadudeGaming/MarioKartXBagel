// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.fasterxml.jackson.databind.AnnotationIntrospector.XmlExtensions;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants.CANIDs;
import frc.robot.Constants.WristConstants.PID;

public class Wrist extends SubsystemBase {

  private final SparkMax zMotor = new SparkMax(CANIDs.zMotor, MotorType.kBrushless);
  private final SparkMax yMotor = new SparkMax(CANIDs.xMotor, MotorType.kBrushless);
  private final SparkMax xMotor = new SparkMax(CANIDs.yMotor, MotorType.kBrushless);

  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  private final PIDController xPID = new PIDController(0.03, 0, 0);
  private final PIDController yPID = new PIDController(0.03, 0, 0);

  private double xPos;
  private double yPos;


  private SparkMaxConfig sparkConfig = new SparkMaxConfig();


  private double desX = 0;
  private double desY = 0;

  /** Creates a new ExampleSubsystem. */
  public Wrist() {
    xMotor.getEncoder().setPosition(0);
    yMotor.getEncoder().setPosition(0);                 
    
    Shuffleboard.getTab(getName()).addDouble("double 1", () -> desX);
    Shuffleboard.getTab(getName()).add("yPID", yPID);
    Shuffleboard.getTab(getName()).add("xPID", xPID);

Shuffleboard.getTab(getName()).addDouble("xIn", () -> xPos);

Shuffleboard.getTab(getName()).addDouble("yIn", () -> yPos);

  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setAxes(controller.getLeftY()*3, controller.getRightX());

    runPID();

    // xPos = xMotor.getEncoder.getPosition() - yMotor.getEncoder.getPosision
    // Shuffleboard.getTab(Constants.OperatorConstants.AUTO_SHUFFLEBOARD).addDouble(getName(), () -> getYaxis());
  }


  public void setAxes (double xIn, double yIn){
    if(Math.abs(xIn) < 0.07) {
      xIn = 0;
    }

    if(Math.abs(yIn) < 0.07) {
      yIn = 0;
    }

    xPos = xIn;
    yPos = yIn;

    double out1 = xIn - yIn;
    double out2 = xIn + yIn;

    desX += out1;
    desY += out2;

    xPID.setSetpoint(out1*10);
    yPID.setSetpoint(out2*10);

  
    // yMotor.set(out2);
  }


  private void runPID(){
    yMotor.set(yPID.calculate(yMotor.getEncoder().getPosition()));
    xMotor.set(xPID.calculate(xMotor.getEncoder().getPosition()));
  }

  public double getXaxis(){
    return ((xMotor.getEncoder().getPosition() + yMotor.getEncoder().getPosition())* 1/25 * 1/1.20833 *360);
  }

  public double getYaxis(){
    return ((yMotor.getEncoder().getPosition() - yMotor.getEncoder().getPosition()) * 1/25 * 1/1.5  *360);
  }

  private double convertSparkAnalogToDegree(double input) {
    // 107.68770565360454681423870774753 is the exact value, but idk if double can fit that much lol
    return input * 107.68770565360455;
  }


  

  public Command runAxes(double in1, double in2){
    return runEnd(
      () -> {
        setAxes(controller.getLeftY(), controller.getRightY());
        // System.out.print("test");
      }, () -> {
        xMotor.set(0);
        yMotor.set(0);
      });
  }
}
