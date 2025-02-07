// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // declare the configuration directory
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");

  // create a swerveDrive object but don't define it yet becasue it coomplains about not handling potential errors
  SwerveDrive swerveDrive;


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
  }


  // command for zeroing the gyro, it needs disabling and re-enabling to start moving again after calling, might want to look into that
  public Command zeroGyro() {
    return run( () -> {
      swerveDrive.zeroGyro();
    });
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
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

  
  // TODO: add remaining functions for pathplanner to get information and add pathplanner
}