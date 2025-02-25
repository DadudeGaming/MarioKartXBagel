// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // declare the configuration directory
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");

  // create a swerveDrive object but don't define it yet becasue it coomplains about not handling potential errors
  SwerveDrive swerveDrive;

  private Vision vision;


  public SwerveSubsystem() {

    // Set Telemetry Verbosity (might want lower for comps as it can slow things down if it's too high, but for testing we don't care)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // just put this code in a try/catch since java complains that there *might* be an error
    try
    {
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }


    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(true);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    setupPathPlanner();  

    setupPhotonVision();

    Shuffleboard.getTab(OperatorConstants.AUTO_SHUFFLEBOARD).add(swerveDrive.field);
  }


  private void setupPhotonVision(){
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }


  public void periodic(){
    SmartDashboard.putData("Field", swerveDrive.field);
    swerveDrive.updateOdometry();
    vision.updatePoseEstimation(swerveDrive);
  }


  // command for zeroing the gyro, it needs disabling and re-enabling to start moving again after calling, might want to look into that
  public Command zeroGyro() {
    return run( () -> {
      swerveDrive.zeroGyro();
    });
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // Methods for actually moving the motors, gets the velocity from the swerve input streams
  public void driveFieldOriented(ChassisSpeeds velcocity) {
    swerveDrive.driveFieldOriented(velcocity);
  }

  // A command that just runs the function above
  public Command driveFieldOriented(Supplier<ChassisSpeeds> Velocity){
    return run(()-> {
      swerveDrive.driveFieldOriented(Velocity.get());
    });
  }

  
  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(3, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(3, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

}