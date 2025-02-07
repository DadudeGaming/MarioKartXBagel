// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // create a new swerve subsystem object
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // create an object for our driver controller
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CanandEventLoop.getInstance();
    // Configure the trigger bindings
    configureBindings();

    // set the default command for the drivebase to the drive command
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                () -> driverController.getLeftY() * 1,
                                                                () -> driverController.getLeftX() * 1)
                                                                .withControllerRotationAxis(driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
                                                                .scaleRotation(OperatorConstants.ROTATION_SCALE)
                                                                .allianceRelativeControl(true);

  // For the right stick to correspond to the angle we want the robot to face instead of the speed of rotationa
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                                                             .headingWhile(true);
  

  // create a new command that calls the driveCommand that we made in the swerveSubsystem
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // Same thing but for direct angle rather than angular velocity
  Command driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);
  

  // define what buttons do on the controller
  private void configureBindings() {
    driverController.button(1).onTrue(drivebase.zeroGyro()); //zero the gyro when square(?) is pressed
  }


  public Command getAutonomousCommand() {
    return null; // this should do things, but that's a later issue :)
  }
}