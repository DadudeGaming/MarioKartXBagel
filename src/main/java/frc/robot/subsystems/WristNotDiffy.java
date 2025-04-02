// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Vector;

import org.dyn4j.geometry.Vector2;

import com.fasterxml.jackson.databind.AnnotationIntrospector.XmlExtensions;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Main;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.CANIDs;

public class WristNotDiffy extends SubsystemBase {

  // private final SparkMax zMotor = new SparkMax(CANIDs.zMotor, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(CANIDs.xMotor, MotorType.kBrushless);
  private final SparkMax MainMotor = new SparkMax(CANIDs.yMotor, MotorType.kBrushless);

  private final CommandPS5Controller controller = new CommandPS5Controller(1);

  // private final PIDController xPID = new PIDController(0.01, 0, 0);
  // private final PIDController yPID = new PIDController(0.01, 0, 0);
  private final ProfiledPIDController wristPID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(40, 35));

  /** Creates a new ExampleSubsystem. */
  public WristNotDiffy() {

    Shuffleboard.getTab(getName()).addDouble("x enc", () -> MainMotor.getEncoder().getPosition());
    Shuffleboard.getTab(getName()).addDouble("y enc", () -> followerMotor.getEncoder().getPosition());


    Shuffleboard.getTab(getName()).addDouble("Wrist Output", () -> wristPID.calculate(MainMotor.getEncoder().getPosition()));


    
    followerMotor.configure(new SparkMaxConfig().follow(MainMotor.getDeviceId(),true), null, PersistMode.kPersistParameters);

    Shuffleboard.getTab(getName()).add("wrist PID", wristPID);

    wristPID.setGoal(MainMotor.getEncoder().getPosition());

    wristPID.setTolerance(WristConstants.PID.precision);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setAxes(controller.getLeftY()*3, controller.getRightX());

    if(DriverStation.isDisabled()){
      wristPID.setGoal(MainMotor.getEncoder().getPosition());

    }

    runPID();

    // xPos = xMotor.getEncoder.getPosition() - yMotor.getEncoder.getPosision
    // Shuffleboard.getTab(Constants.OperatorConstants.AUTO_SHUFFLEBOARD).addDouble(getName(), () -> getYaxis());
  }


  public void setSetpoint(double inSetpoint) {
    wristPID.setGoal(inSetpoint);
  }

  private void runPID(){
    followerMotor.set(wristPID.calculate(MainMotor.getEncoder().getPosition()));
    MainMotor.set(wristPID.calculate(MainMotor.getEncoder().getPosition()));
    // zMotor.set(zPID.calculate(zMotor.getEncoder().getPosition()));
  }

  public boolean isAtGoal() {
    return wristPID.atGoal();
  }


  

  
}
