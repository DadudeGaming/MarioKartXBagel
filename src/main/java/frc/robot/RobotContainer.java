// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArduinoLEDSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


import com.reduxrobotics.canand.CanandEventLoop;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



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
  
  // create a new pivot subystem object
  private final ArmSubsystem arm = new ArmSubsystem();

  public final ClimbSubsystem climb = new ClimbSubsystem();

  private final PowerDistribution pdh = new PowerDistribution(10, ModuleType.kRev);
 
  // create an object for our driver controller
  // private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandPS5Controller driverController = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(Constants.OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;


  private boolean directAngle = false;

  private ArduinoLEDSubsystem LEDSystem = new ArduinoLEDSubsystem();

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

    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Match Time", () -> Timer.getMatchTime());
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Voltage", () -> pdh.getVoltage());
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Current", () -> pdh.getTotalCurrent());
    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Power", () -> pdh.getTotalPower());
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

    /** Set up the commands to change the pivot position */
    // driverController.button(5).onTrue(new ArmCommand(pivotSubsystem, 0));
    // driverController.button(6).onTrue(new ArmCommand(pivotSubsystem, 1));
    // driverController.button(7).onTrue(new ArmCommand(pivotSubsystem, 2));
    // driverController.button(8).onTrue(new ArmCommand(pivotSubsystem, 3));
    // driverController.button(9).onTrue(new ArmCommand(pivotSubsystem, 4));

    driverController.L1().whileTrue(arm.decreaseSetpoint());
    driverController.R1().whileTrue(arm.increaseSetpoint());


    driverController.button(1).whileTrue(drivebase.zeroGyro()); //zero the gyro when square(?) is pressed
                              

    driverController.circle().whileTrue(Commands.none());

    driverController.povDown().whileTrue(climb.outake());

    driverController.povUp().and(() -> (climb.getEncoder() < 60)).whileTrue(climb.intake());
    }

  private void setupAutoChooser(){
    new PathPlannerAuto("Test Auto");
    new PathPlannerAuto("AL4 HL4");
    new PathPlannerAuto("JL4 HL4");
    new PathPlannerAuto("Just Leave");



    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}