// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class EndEffectorSubsystem extends SubsystemBase {

  // declare variables up here, you can either set their value here, or later in the code, it mostly just depends on what you're doing

  // initialize a variable for keeping track of the intake state
  public boolean intakeMode; 

  private final Canandcolor colorSense = new Canandcolor(22);

  public final TalonFX intakeMotor = new TalonFX(IntakeConstants.CANID);

  /** Creates a new ExampleMotorSubsystem. */
  public EndEffectorSubsystem() {
    intakeMode = false; // default to ourtake first (we have a preloaded game piece)
    // start with factory-default configs
    var currentConfigs = new MotorOutputConfigs();

    // The forward motor (probably) goes forward
    currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
    // intakeMotor.getConfigurator().apply(currentConfigs);

    Shuffleboard.getTab(getName()).addBoolean("Intake Mode", () -> intakeMode);
    Shuffleboard.getTab(getName()).addDouble("Dist", () -> colorSense.getProximity());

    Shuffleboard.getTab(getName()).addBoolean("Coral Dectected", () -> getDist() < 0.35);
    // Shuffleboard.getTab(getName()).addDouble("Motor Current", () -> intakeMotor.getTorqueCurrent().getValueAsDouble());
  }

  public double getDist(){
    return colorSense.getProximity();
  }



  // Commands can either run
  // public Command runMotor() {
  //   // depending on what the intake bool is, return either the intake or outake command.
  //   // when we actually make the robot, we might want to use a sensor to detect if there's a coral in the robot already instead of just toggling
  //   if(!intakeMode){
  //     return new OuttakeCommand(this, intakeMode).alongWith(new PrintCommand(String.valueOf(intakeMode))); 
  //     // return new PrintCommand(String.valueOf(intakeMode)); // return the correct command to run, with a 3 second timeout, so it stops after 3 seconds
  //   } else {
  //     // return new IntakeCommand(this, true).withTimeout(1.5);
  //     return new PrintCommand(String.valueOf(intakeMode));
  //   }
  // }

  public void setMotor(double in){
    intakeMotor.setControl(new DutyCycleOut(in));
  }

  public boolean isStalled() {
    if(intakeMotor.getTorqueCurrent().getValueAsDouble() > IntakeConstants.stopCurrent){
      return true;
    }
    return false;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run, constantly, usually you'll put logging stuff here
  }
}
