// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ArmAngles;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_forwardMotor = new TalonFX(ArmConstants.CANIDs.frontMotorCANID);
  private final TalonFX m_backwardMotor = new TalonFX(ArmConstants.CANIDs.backMotorCANID);

  private final DutyCycleOut m_forwardOut = new DutyCycleOut(0);
  Canandmag canandmag = new Canandmag(ArmConstants.CANIDs.canandmagCANID);
  
  private boolean isInputing;
  public final PIDController forwardController = new PIDController(ArmConstants.PIDConstants.kP, ArmConstants.PIDConstants.kI, ArmConstants.PIDConstants.kD);

  public final ProfiledPIDController profiledArmController = new ProfiledPIDController(ArmConstants.PIDConstants.kP, ArmConstants.PIDConstants.kI, ArmConstants.PIDConstants.kD, new TrapezoidProfile.Constraints(600, 300));
  
  private double output;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // USED TO PERMENENTLY ZERO THE ENCODER
    // KEEP COMMENTED
    //canandmag.setAbsPosition(0.04);

    // start with factory-default configs
    var currentConfigs = new MotorOutputConfigs();

    // The forward motor (probably) goes forward
    currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
    m_forwardMotor.getConfigurator().apply(currentConfigs);

    // The backward motor is just the reverse of the forward one
    m_backwardMotor.setControl(new Follower(m_forwardMotor.getDeviceID(), true));

    forwardController.setSetpoint(encoderInDegrees());
    profiledArmController.setGoal(encoderInDegrees());


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("Arm PID", forwardController);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("ProfiledPID", profiledArmController);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("ForwardOutput", () -> m_forwardOut.Output);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Encoder", () -> encoderInDegrees());
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addBoolean("Inputing", () -> isInputing);
  }


  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(DriverStation.isDisabled()){
      profiledArmController.setGoal(encoderInDegrees());
    }

    runPID();
  }

  public void setArmAngle(double input) {
    // if (encoderInDegrees() > ArmConstants.positions.minPos && encoderInDegrees() < ArmConstants.positions.maxPos) {
         // Should be (encoderValue, then setpoint)
        //  forwardController.setSetpoint(input);
        profiledArmController.setGoal(input);
         
     //}
 }

 public double encoderInDegrees(){
  return canandmag.getAbsPosition() * 360;
 }

 public void runPID() {
    //  output = forwardController.calculate(encoderInDegrees());
    output = profiledArmController.calculate(encoderInDegrees(), profiledArmController.getGoal());
    //  if (output > 0.75)
    //  output = 0.75;

    //  if (output < -0.75)
    //      output = -0.75;

     m_forwardOut.Output = output;
     m_forwardMotor.setControl(new DutyCycleOut(output));
 }

 public Command increaseSetpoint(){
  return runEnd(() -> {
    if(forwardController.getSetpoint() < ArmConstants.ArmAngles.L4){
      var curSetpoint = forwardController.getSetpoint() + 0.05;
      if(curSetpoint > ArmAngles.Stowed){
           curSetpoint = ArmAngles.Stowed;
      }
      forwardController.setSetpoint(curSetpoint);
    }
    isInputing = true;
    // if(encoderInDegrees() < 90){
    // m_forwardOut.Output = 0.05;
    // }
    // else{
    //   m_forwardOut.Output = 0;
    // }
    // m_forwardMotor.setControl(m_forwardOut);
  },
  () -> {
    // m_forwardOut.Output = 0;
    // m_forwardMotor.setControl(m_forwardOut);
    forwardController.setSetpoint(encoderInDegrees());
  }
  );
 }

 public Command decreaseSetpoint(){
  return runEnd(() -> {
    if(forwardController.getSetpoint() > ArmConstants.ArmAngles.Stowed){
      var curSetpoint = forwardController.getSetpoint() - 0.05;
      if(curSetpoint < ArmAngles.L4){
        curSetpoint = ArmAngles.L4;
      }
      forwardController.setSetpoint(curSetpoint);
    }
    isInputing = true;
    // if (encoderInDegrees() > 20){
    // m_forwardOut.Output = -0.05;
    // } else {
    //   m_forwardOut.Output = 0;
    // }
    // m_forwardMotor.setControl(m_forwardOut);
  },
  () -> {
    // m_forwardOut.Output = 0;
    // m_forwardMotor.setControl(m_forwardOut);
    forwardController.setSetpoint(encoderInDegrees());
  }
  );
 }
}
