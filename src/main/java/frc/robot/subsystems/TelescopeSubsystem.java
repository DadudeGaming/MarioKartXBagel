// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Constants.OperatorConstants;

public class TelescopeSubsystem extends SubsystemBase {

  

  
  public final TalonFX telescopeMotor = new TalonFX(TelescopeConstants.MotorCANID);

  public final DutyCycleOut motorOut = new DutyCycleOut(0);

  
  public final ProfiledPIDController telescopePID = new ProfiledPIDController(TelescopeConstants.PIDConstants.kP, TelescopeConstants.PIDConstants.kI, TelescopeConstants.PIDConstants.kD, new TrapezoidProfile.Constraints(TelescopeConstants.PIDConstants.kMaxVel, TelescopeConstants.PIDConstants.kMaxAccel));


  /** Creates a new ArmSubsystem. */
  public TelescopeSubsystem() {

    // start with factory-default configs
    var currentConfigs = new MotorOutputConfigs();

    // The forward motor (probably) goes forward
    currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    telescopeMotor.getConfigurator().apply(currentConfigs);

    

  
    telescopePID.setGoal(encoderInCM());
    telescopePID.setTolerance(TelescopeConstants.precisionInCM);


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("telescope PID", telescopePID);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Telescope Output", () -> motorOut.Output);
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Telescope Position", () -> encoderInCM());
  }


  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Constantly set the setpoint to the current position so it doesn't destroy any more garage doors
    if(DriverStation.isDisabled()){
      telescopePID.setGoal(encoderInCM());
    }

    runPID();
  }

  public void setTelescopeLength(double input) {
    telescopePID.setGoal(input);
 }

 public double encoderInCM(){
  return (telescopeMotor.getPosition().getValueAsDouble() / 25) * (64.8 * Math.PI);
 }

 public boolean isAtGoal() {
  return telescopePID.atGoal();
 }

 public void runPID() {
    
    motorOut.Output = telescopePID.calculate(encoderInCM(), telescopePID.getGoal());
    telescopeMotor.setControl(motorOut);
 }
}