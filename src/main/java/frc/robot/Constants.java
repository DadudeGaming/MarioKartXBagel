// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadbands
    public static final double  DEADBAND           = 0.07;

    // Scales for movement and rotation (1 is full speed)
    public static final double  TRANSLATION_SCALE  = 0.8;
    public static final double  ROTATION_SCALE     = 0.8;

    // if we want field centric control or not (is forwards on the controller always away from  the driver station or is it forwards for the robot)
    public static final boolean FIELD_CENTRIC      = true;

    public static final String AUTO_SHUFFLEBOARD = "AUTO";
  }


  // max speed of the robot in m/s use Units.feetToMeters to use feet
  public static final   double  MAX_SPEED = 4.5;

  public static final double reefOffset = 0.165;

  public static final double reefOffsetY = 0.56;



  public static class VisionConstants{
    public static final String kFrontCameraName = "Camera-1";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // pitched upward.
    private static final double camPitch = Units.degreesToRadians(30.0);

    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.2, 0.2, 0.51), new Rotation3d(0, -camPitch, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
