// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Vector;

import org.dyn4j.geometry.Vector2;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.CANIDs;

public class Wrist extends SubsystemBase {

  // private final SparkMax zMotor = new SparkMax(CANIDs.zMotor, MotorType.kBrushless);
  private final SparkMax yMotor = new SparkMax(CANIDs.xMotor, MotorType.kBrushless);
  private final SparkMax xMotor = new SparkMax(CANIDs.yMotor, MotorType.kBrushless);

  private final CommandPS5Controller controller = new CommandPS5Controller(1);

  // private final PIDController xPID = new PIDController(0.01, 0, 0);
  // private final PIDController yPID = new PIDController(0.01, 0, 0);
  private final ProfiledPIDController xPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(27, 23));
  private final ProfiledPIDController yPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(27, 23));
  private final PIDController zPID = new PIDController(0.03, 0, 0);

  private double xPos;
  private double yPos;


  private SparkMaxConfig sparkConfig = new SparkMaxConfig();


  private double desX = 0;
  private double desY = 0;

  /** Creates a new ExampleSubsystem. */
  public Wrist() {
    // xMotor.getEncoder().setPosition(-33.5);
    // yMotor.getEncoder().setPosition(-15.5);
    // zMotor.getEncoder().setPosition(0);
    
    // Shuffleboard.getTab(getName()).addDouble("zencoder", () -> zMotor.getEncoder().getPosition());

    Shuffleboard.getTab(getName()).addDouble("x setpoint", () -> controller.getRightX()*2);
    Shuffleboard.getTab(getName()).addDouble("y setpoint", () -> controller.getLeftY());

    Shuffleboard.getTab(getName()).addDouble("x enc", () -> xMotor.getEncoder().getPosition());
    Shuffleboard.getTab(getName()).addDouble("y enc", () -> yMotor.getEncoder().getPosition());


    Shuffleboard.getTab(getName()).add("yPID", yPID);
    Shuffleboard.getTab(getName()).add("xPID", xPID);
    Shuffleboard.getTab(getName()).add("zPID", zPID);

    Shuffleboard.getTab(getName()).addDouble("xIn", () -> xPos);

    Shuffleboard.getTab(getName()).addDouble("yIn", () -> yPos);

    yPID.setGoal(yMotor.getEncoder().getPosition());
    xPID.setGoal(xMotor.getEncoder().getPosition());

    yPID.setTolerance(WristConstants.PID.precision);
    xPID.setTolerance(WristConstants.PID.precision);

    zPID.setSetpoint(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setAxes(controller.getLeftY()*3, controller.getRightX());

    if(DriverStation.isDisabled()){
      yPID.setGoal(yMotor.getEncoder().getPosition());
      xPID.setGoal(xMotor.getEncoder().getPosition());
    }

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

    xPID.setGoal(out1*10);
    yPID.setGoal(out2*10);

  
    // yMotor.set(out2);
  }

  public void setPID(Vector2 input) {
    xPID.setGoal(input.x);
    yPID.setGoal(input.y);
  }


  private void runPID(){
    yMotor.set(yPID.calculate(yMotor.getEncoder().getPosition()));
    xMotor.set(xPID.calculate(xMotor.getEncoder().getPosition()));
    // zMotor.set(zPID.calculate(zMotor.getEncoder().getPosition()));
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




  public boolean isAtGoal() {
    return xPID.atGoal() && yPID.atGoal();
  }


  

  public Command runAxes(double in1, double in2){
    return runEnd(
      () -> {
        setAxes(controller.getRightX(), controller.getLeftY()*2);
        // System.out.print("test");
      }, () -> {
        xMotor.set(0);
        yMotor.set(0);
      });
  }
}
