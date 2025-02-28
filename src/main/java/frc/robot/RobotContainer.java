// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


import com.reduxrobotics.canand.CanandEventLoop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  // private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandPS5Controller driverController = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(Constants.OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;
  // Build an auto chooser. This will use Commands.none() as the default option.

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CanandEventLoop.getInstance();
    // Configure the trigger bindings
    configureBindings();

    // Shut up
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    setupAutoChooser();

    // set the default command for the drivebase to the drive command
    // drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
                                                                .scaleRotation(-OperatorConstants.ROTATION_SCALE)
                                                                .allianceRelativeControl(true);

  // For the right stick to correspond to the angle we want the robot to face instead of the speed of rotationa
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                                                                                            () -> driverController.getRightX() * -1,
                                                                                            () -> driverController.getRightY() *-1)
                                                                                             .headingWhile(true);
  

  // create a new command that calls the driveCommand that we made in the swerveSubsystem
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // Same thing but for direct angle rather than angular velocity
  Command driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);
  

  // define what buttons do on the controller
  private void configureBindings() {
    driverController.button(1).whileTrue(drivebase.zeroGyro()); //zero the gyro when square(?) is pressed


    driverController.triangle()
                              .toggleOnFalse(driveFieldOrientedDirectAngle);
                              

    driverController.circle().whileTrue(Commands.none());
  }

  private void setupAutoChooser(){
    new PathPlannerAuto("Test Auto");
    new PathPlannerAuto("AL4 HL4");
    new PathPlannerAuto("JL4 HL4");


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}