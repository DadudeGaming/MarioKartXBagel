// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.BumperAddressableLED;

// import frc.robot.subsystems.SwerveSubsystem;


// import frc.robot.commands.ArmCommand;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // create a new swerve subsystem object
  // private final SwerveSubsystem drivebase = new SwerveSubsystem();


  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  
  private final BumperAddressableLED m_BumperAddressableLED = new BumperAddressableLED();

  // private final ClimbCamera climbCamera = new ClimbCamera();
 
  // create an object for our driver controller
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  
  private final CommandPS5Controller operatorController = new CommandPS5Controller(Constants.OperatorConstants.kOperatorControllerPort);


  private final SendableChooser<Command> autoChooser;


  // Declare a variable to track the current speed
  public static double currentSpeed = 0.0;

  // Define constants for acceleration, deceleration, and decay
  private static final double ACCELERATION_RATE = 0.05; // Rate of increase when "b" is pressed
  private static final double DECELERATION_RATE = 0.05; // Rate of decrease when "a" is pressed
  private static final double DECAY_RATE = 0.02;        // Rate of decay when no button is pressed
  

  // Build an auto chooser. This will use Commands.none() as the default option.

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CanandEventLoop.getInstance();

    m_BumperAddressableLED.pattern1();
                                                                  
                    
    // NamedCommands.registerCommand("Score", new SequentialCommandGroup(

    // Configure the trigger bindings
    configureBindings();

    // Increase speed while "b" is pressed
  driverController.b().whileTrue(Commands.run(() -> {
    currentSpeed = Math.min(currentSpeed + ACCELERATION_RATE, 1.0); // Cap speed at 1.0
  }));

  // Decrease speed while "a" is pressed
  driverController.a().whileTrue(Commands.run(() -> {
    currentSpeed = Math.max(currentSpeed - DECELERATION_RATE, -0.4); // Cap speed at -1.0
  }));  

    // Shut up
    // DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    setupAutoChooser();

    // set the default command for the drivebase to the drive command
    drivebase.setDefaultCommand(driveFieldOrientedWithDecay); //driveFieldOrientedAngularVelocity


    // wrist.setDefaultCommand(wrist.runAxes(operatorController.getRightX(), operatorController.getLeftY()));
  
    drivebase.setDefaultCommand(driveFieldOrientedWithDecay);

    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Match Time", () -> Timer.getMatchTime());
    // Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Voltage", () -> pdh.getVoltage());
    // Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Current", () -> pdh.getTotalCurrent());
    // Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).addDouble("Power", () -> pdh.getTotalPower());
    
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                () -> currentSpeed,
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

  // // create a new command that calls the driveCommand that we made in the swerveSubsystem
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // // Same thing but for direct angle rather than angular velocity
  Command driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);

    // Wrap the existing driveFieldOrientedAngularVelocity command with additional logic
  Command driveFieldOrientedWithDecay = Commands.parallel(
    // Existing drive command
    driveFieldOrientedAngularVelocity,

    // Deceleration logic and SmartDashboard updates
    Commands.run(() -> {
        // Decay speed slowly when neither button is pressed
        if (!driverController.b().getAsBoolean() && !driverController.a().getAsBoolean()) {
            if (currentSpeed > 0) {
                currentSpeed = Math.max(currentSpeed - DECAY_RATE, 0); // Decay toward 0
            } else if (currentSpeed < 0) {
                currentSpeed = Math.min(currentSpeed + DECAY_RATE, 0); // Decay toward 0
            }
        }

        // Update SmartDashboard with the current speed
        SmartDashboard.putNumber("Current Speed", currentSpeed);
    })
  );
  

  // define what buttons do on the controller
  private void configureBindings() { //circle accel x decell
    // /** Set up the commands to change the pivot position */

    //driverController.x().onTrue(m_BumperAddressableLED.setWhiteCommand());

    // driverController.R1().and(() -> stateManager.robotState != "STOWED").onTrue(new TelescopeCommand(telescope, 0)
    //                                 // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                                 .andThen(new WristCommand(wrist, 0))
    //                                 .andThen(new ArmCommand(arm, 0))
    //                                 .andThen(stateManager.setRobotState("STOWED")));
                                  
    // driverController.square().and(() -> stateManager.robotState != "L1").onTrue(new TelescopeCommand(telescope, 0)
    //                                     // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                                     .andThen(new ArmCommand(arm, 1))
    //                                     .andThen(new TelescopeCommand(telescope, 1).alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 1))
    //                                     .andThen(stateManager.setRobotState("L1")))));

    // driverController.cross().and(() -> stateManager.robotState != "L2").onTrue(new TelescopeCommand(telescope, 0)
    //                                     // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                                     .andThen(new ArmCommand(arm, 2))
    //                                     .andThen(new TelescopeCommand(telescope, 2).alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 2))
    //                                     .andThen(stateManager.setRobotState("L2")))));

    // driverController.circle().and(() -> stateManager.robotState != "L3").onTrue(new TelescopeCommand(telescope, 0)
    //                                     // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                                     .andThen(new ArmCommand(arm, 3))
    //                                     .andThen(new TelescopeCommand(telescope, 3).alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 3))
    //                                     .andThen(stateManager.setRobotState("L3")))));

    // driverController.triangle().and(() -> stateManager.robotState != "CLIMB").onTrue(new TelescopeCommand(telescope, 0)
    //                                     // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                                     .andThen(new ArmCommand(arm, 4))
    //                                     .andThen(new TelescopeCommand(telescope, 4).alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 4))
    //                                     .andThen(stateManager.setRobotState("CLIMB")))));

    

    // operator stuff                              
    // operatorController.R1().onTrue(new TelescopeCommand(telescope, 0)
    //                                 .alongWith(new WristCommand(wrist, 0))
    //                                 .andThen(new ArmCommand(arm, 0)));
                                  
    // operatorController.square().onTrue(new TelescopeCommand(telescope, 0)
    //                                     .alongWith(new WristCommand(wrist, 0))
    //                                     .andThen(new ArmCommand(arm, 1))
    //                                     .andThen(new TelescopeCommand(telescope, 1))
    //                                     .alongWith(new WristCommand(wrist, 1)));

    // operatorController.cross().onTrue(new TelescopeCommand(telescope, 0)
    //                                     .alongWith(new WristCommand(wrist, 0))
    //                                     .andThen(new ArmCommand(arm, 2))
    //                                     .andThen(new TelescopeCommand(telescope, 2))
    //                                     .alongWith(new WristCommand(wrist, 2)));

    // operatorController.circle().onTrue(new TelescopeCommand(telescope, 0)
    //                                     .alongWith(new WristCommand(wrist, 0))
    //                                     .andThen(new ArmCommand(arm, 3))
    //                                     .andThen(new TelescopeCommand(telescope, 3))
    //                                     .alongWith(new WristCommand(wrist, 3)));

    // operatorController.triangle().onTrue(new TelescopeCommand(telescope, 0)
    //                                     .alongWith(new WristCommand(wrist, 0))
    //                                     .andThen(new ArmCommand(arm, 4))
    //                                     .andThen(new TelescopeCommand(telescope, 4))
    //                                     .alongWith(new WristCommand(wrist, 4)));

    // driverController.R1().onTrue(new TelescopeCommand(telescope, 0));
    // driverController.square().onTrue(new TelescopeCommand(telescope, 1));
    // driverController.cross().onTrue(new TelescopeCommand(telescope, 2));
    // driverController.circle().onTrue(new TelescopeCommand(telescope, 3));
    // driverController.triangle().onTrue(new TelescopeCommand(telescope, 4));

    // driverController.R1().onTrue(new WristCommandDirectAxes(wrist, 0));
    // driverController.square().onTrue(new WristCommandDirectAxes(wrist, 1));
    // driverController.cross().onTrue(new WristCommandDirectAxes(wrist, 2));
    // driverController.circle().onTrue(new WristCommandDirectAxes(wrist, 3));
    // driverController.triangle().onTrue(new WristCommandDirectAxes(wrist, 4));

    // driverController.R1().onTrue(new ArmCommand(arm, 0));
    // driverController.square().onTrue(new ArmCommand(arm, 1));
    // driverController.cross().onTrue(new ArmCommand(arm, 2));
    // driverController.circle().onTrue(new ArmCommand(arm, 3));
    // driverController.triangle().onTrue(new ArmCommand(arm, 4));

    //driverController.L1().onTrue(new OuttakeCommand(intake).withTimeout(1.5)); //Commented

    // operatorController.L2().whileTrue(wrist.runAxes(operatorController.getRightX(), operatorController.getLeftY()));
                                                            


    // driverController.R1().and(() -> stateManager.robotState != 6).onTrue(stateManager.goToState(0, telescope, arm, wrist));

    // driverController.square().and(() -> stateManager.robotState != 6).onTrue(stateManager.goToState(1, telescope, arm, wrist));
    // driverController.cross().and(() -> stateManager.robotState != 6).onTrue(stateManager.goToState(2, telescope, arm, wrist));
    // driverController.circle().and(() -> stateManager.robotState != 6).onTrue(stateManager.goToState(3, telescope, arm, wrist));
    // driverController.triangle().and(() -> stateManager.robotState != 6).onTrue(stateManager.goToState(4, telescope, arm, wrist));

    // driverController.R1().and(() -> stateManager.robotState != 6).onTrue(new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))).andThen(stateManager.goToState(0, telescope, arm, wrist)));

    // driverController.square().and(() -> stateManager.robotState == 6).onTrue(new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))).andThen(stateManager.goToState(1, telescope, arm, wrist)));
    // driverController.cross().and(() -> stateManager.robotState == 6).onTrue(new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))).andThen(stateManager.goToState(2, telescope, arm, wrist)));
    // driverController.circle().and(() -> stateManager.robotState == 6).onTrue(new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))).andThen(stateManager.goToState(3, telescope, arm, wrist)));
    // driverController.triangle().and(() -> stateManager.robotState == 6).onTrue(new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))).andThen(stateManager.goToState(4, telescope, arm, wrist)));
    
    
    // driverController.R1().onTrue(stateManager.goToState(0, telescope, arm, wrist)); //Commented

    // driverController.square().onTrue(stateManager.goToState(1, telescope, arm, wrist));
    // driverController.cross().onTrue(stateManager.goToState(2, telescope, arm, wrist));
    // driverController.circle().onTrue(stateManager.goToState(3, telescope, arm, wrist));
    // driverController.triangle().onTrue(stateManager.goToState(5, telescope, arm, wrist)); //to here

    // driverController.R2().and(() -> intake.intakeMode).onTrue(new IntakeCommand(intake).until(() -> driverController.L2().getAsBoolean()));
    // driverController.R2().and(() -> !intake.intakeMode).onTrue(new OuttakeCommand(intake).withTimeout(0.6));

    // intake
    // driverController.R2().and(() -> stateManager.robotState != 6).onTrue(
    //                             new TelescopeCommand(telescope, 0)
    //                               // .alongWith(new WristCommand(wrist, 6).andThen(new WristCommand(wrist, 0)))
    //                               .andThen(new ArmCommand(arm, 6))
    //                               .andThen(new TelescopeCommand(telescope, 6).andThen(new WristCommand(wrist, 7)
    //                               .andThen(new IntakeCommand(intake).until(driverController.R3()))
    //                               .andThen(stateManager.setRobotState(6)))));

    // driverController.R2().onTrue(stateManager.goToState(6, telescope, arm, wrist));
    // driverController.R2().onTrue(new SequentialCommandGroup(stateManager.goToState(6, telescope, arm, wrist), //Commented
    //                              new IntakeCommand(intake).until(driverController.R3())));
    
    // driverController.triangle().onTrue(new SequentialCommandGroup(stateManager.goToState(5, telescope, arm, wrist),
    //                               new IntakeCommand(intake).until(driverController.R3()))); //to here

  // driverController.R2().and(() -> stateManager.robotState == 6).onTrue(
  //                                     new IntakeCommand(intake).until(driverController.R3()));
    
    //driverController.L2().onTrue(new LowerCommand(arm, telescope)); //Commented

    // operatorController.R2().and(() -> intake.intakeMode).onTrue(new IntakeCommand(intake).until(() -> driverController.L2().getAsBoolean()));
    // operatorController.R2().and(() -> !intake.intakeMode).onTrue(new OuttakeCommand(intake).withTimeout(0.6));
    //driverController.L3().whileTrue(drivebase.zeroGyro()); //zero the gyro when square(?) is pressed //Commented

    // driverController.povDown().whileTrue(climb.outake());

    // driverController.povUp().and(() -> (climb.getEncoder() < 60)).whileTrue(climb.intake());
    }

  private void setupAutoChooser(){
    new PathPlannerAuto("Test Auto");
    new PathPlannerAuto("AL4 HL4");
    new PathPlannerAuto("JL4 HL4");

    new PathPlannerAuto("Top L1");
    new PathPlannerAuto("Middle L1");
    new PathPlannerAuto("Bottom L1");

    

    


    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}