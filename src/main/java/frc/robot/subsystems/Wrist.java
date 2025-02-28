// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants.CANIDs;

public class Wrist extends SubsystemBase {

  private final SparkMax zMotor = new SparkMax(CANIDs.zMotor, MotorType.kBrushless);
  private final SparkMax yMotor = new SparkMax(CANIDs.yMotor, MotorType.kBrushless);
  private final SparkMax xMotor = new SparkMax(CANIDs.xMotor, MotorType.kBrushless);

  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  private final SparkClosedLoopController velocityControllerX = xMotor.getClosedLoopController();

  private SparkMaxConfig sparkConfig = new SparkMaxConfig();

  private final SparkClosedLoopController velocityControllerY = yMotor.getClosedLoopController();

  /** Creates a new ExampleSubsystem. */
  public Wrist() {
    sparkConfig.closedLoop
                          .p(0.0001)
                          .i(0)
                          .d(0)
                          .velocityFF(0.0001);
    
    xMotor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    yMotor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    xMotor.getEncoder().setPosition(0);
    yMotor.getEncoder().setPosition(0);                 
    
    Shuffleboard.getTab(Constants.OperatorConstants.AUTO_SHUFFLEBOARD).addNumber("X Axis", () -> getXaxis());
    Shuffleboard.getTab(Constants.OperatorConstants.AUTO_SHUFFLEBOARD).addNumber("Y Axis", () -> getYaxis());
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Shuffleboard.getTab(Constants.OperatorConstants.AUTO_SHUFFLEBOARD).addDouble(getName(), () -> getYaxis());
  }


  public void setAxes (double xIn, double yIn){
    if(Math.abs(xIn) < 0.07) {
      xIn = 0;
    }

    if(Math.abs(yIn) < 0.07) {
      yIn = 0;
    }

    double out1 = xIn - yIn;
    double out2 = xIn + yIn;

    velocityControllerX.setReference(out1 *1100, ControlType.kVelocity);
    velocityControllerY.setReference(out2 *1100, ControlType.kVelocity);
    // yMotor.set(out2);
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
