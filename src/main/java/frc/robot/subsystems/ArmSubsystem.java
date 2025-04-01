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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_forwardMotor = new TalonFX(ArmConstants.CANIDs.frontMotorCANID);
  private final TalonFX m_backwardMotor = new TalonFX(ArmConstants.CANIDs.backMotorCANID);

  private final DutyCycleOut m_forwardOut = new DutyCycleOut(0);
  Canandmag canandmag = new Canandmag(ArmConstants.CANIDs.canandmagCANID);

  
  public final ProfiledPIDController profiledArmController = new ProfiledPIDController(ArmConstants.PIDConstants.kP, ArmConstants.PIDConstants.kI, ArmConstants.PIDConstants.kD, new TrapezoidProfile.Constraints(ArmConstants.PIDConstants.kMaxVel, ArmConstants.PIDConstants.kMaxAccel));
  
  private double output;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    // start with factory-default configs
    var currentConfigs = new MotorOutputConfigs();

    // The forward motor (probably) goes forward
    currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
    m_forwardMotor.getConfigurator().apply(currentConfigs);

    // The backward motor is just the reverse of the forward one
    m_backwardMotor.setControl(new Follower(m_forwardMotor.getDeviceID(), true));

  
    profiledArmController.setGoal(encoderInDegrees());
    profiledArmController.setTolerance(ArmConstants.precisionInDegrees);


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("ArmPID", profiledArmController);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("ForwardOutput", () -> m_forwardOut.Output);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Encoder", () -> encoderInDegrees());
  }


  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Constantly set the setpoint to the current position so it doesn't destroy any more garage doors
    if(DriverStation.isDisabled()){
      profiledArmController.setGoal(encoderInDegrees());
    }

    runPID();
  }

  public void setArmAngle(double input) {
        profiledArmController.setGoal(input);
 }

 public double encoderInDegrees(){
  return canandmag.getAbsPosition() * 360;
 }

 public boolean isAtGoal() {
  return profiledArmController.atGoal();
 }

 public void runPID() {
    output = profiledArmController.calculate(encoderInDegrees(), profiledArmController.getGoal());
    m_forwardOut.Output = output;
    m_forwardMotor.setControl(m_forwardOut);
 }
}