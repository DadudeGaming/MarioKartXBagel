// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OperatorConstants;

import java.util.function.DoubleSupplier;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public double angle;
  private DoubleSupplier angleDoubleSupplier;

  public boolean up = false;
  public boolean down = false;

  private final CommandPS5Controller driverController = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

  // Please read here:
  /* This is simply a digital subsystem, it's not intended for actual usage, only for simulatating the arm.*/

  public ArmSubsystem() {
    angle = 0.0;
    angleDoubleSupplier = () -> angle;
    Shuffleboard.getTab(OperatorConstants.DRIVER_SHUFFLEBOARD).addNumber("Angle", angleDoubleSupplier);
  }

  public void changeAngle(double input) {
    angle += input*0.01;
    // Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Speaker Down", speakerDownBoolSupplier);
    // Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addNumber("Pivot Output", outputSupplier);
  }
}
