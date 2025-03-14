// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ArmAngles;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_forwardMotor = new TalonFX(ArmConstants.CANIDs.frontMotorCANID);
  private final TalonFX m_backwardMotor = new TalonFX(ArmConstants.CANIDs.backMotorCANID);

  private final DutyCycleOut m_forwardOut = new DutyCycleOut(0);
  Canandmag canandmag = new Canandmag(ArmConstants.CANIDs.canandmagCANID);
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // start with factory-default configs
    var currentConfigs = new MotorOutputConfigs();

    // The forward motor (probably) goes forward
    currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    m_forwardMotor.getConfigurator().apply(currentConfigs);

    // The backward motor is just the reverse of the forward one
    m_backwardMotor.setControl(new Follower(m_backwardMotor.getDeviceID(), true));

    forwardController.setSetpoint(encoderInDegrees());


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add(forwardController);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Encoder", () -> encoderInDegrees());
  }


  public final PIDController forwardController = new PIDController(ArmConstants.PIDConstants.kP, ArmConstants.PIDConstants.kI, ArmConstants.PIDConstants.kD);
  
  private double output;
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    runPID();
  }

  public void setArmAngle(double input) {
    // if (encoderInDegrees() > ArmConstants.positions.minPos && encoderInDegrees() < ArmConstants.positions.maxPos) {
         // Should be (encoderValue, then setpoint)
         forwardController.setSetpoint(input);
         
     //}
 }

 public double encoderInDegrees(){
  return canandmag.getAbsPosition() * 360;
 }

 public void runPID() {
     output = forwardController.calculate(encoderInDegrees());
     if (output > 0.55)
     output = 0.55;

     if (output < -0.55)
         output = -0.55;

     m_forwardOut.Output = output;
    //  m_forwardMotor.setControl(m_forwardOut);
 }

 public Command increaseSetpoint(){
  return run(() -> {
    // if(forwardController.getSetpoint() < ArmConstants.ArmAngles.L4){
    //   var curSetpoint = forwardController.getSetpoint() + 0.05;
    //   if(curSetpoint > ArmAngles.L4){
    //        curSetpoint = ArmAngles.L4;
    //   }
    //   forwardController.setSetpoint(curSetpoint);
    // }
    m_forwardOut.Output = 0.05;
    m_forwardMotor.setControl(m_forwardOut);
  });
 }

 public Command decreaseSetpoint(){
  return run(() -> {
    // if(forwardController.getSetpoint() > ArmConstants.ArmAngles.Stowed){
    //   var curSetpoint = forwardController.getSetpoint() - 0.05;
    //   if(curSetpoint < ArmAngles.Stowed){
    //     curSetpoint = ArmAngles.Stowed;
    //   }
    //   forwardController.setSetpoint(curSetpoint);
    // }

    m_forwardOut.Output = -0.05;
    m_forwardMotor.setControl(m_forwardOut);
  });
 }
}
