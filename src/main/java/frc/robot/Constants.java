// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadbands
    public static final double  DEADBAND           = 0.05;

    // Scales for movement and rotation (1 is full speed)
    public static final double  TRANSLATION_SCALE  = 1;
    public static final double  ROTATION_SCALE     = 1;

    // if we want field centric control or not (is forwards on the controller always away from  the driver station or is it forwards for the robot)
    public static final boolean FIELD_CENTRIC      = true;

    public static final String AUTO_SHUFFLEBOARD = "AUTO";
    public static final String DRIVER_SHUFFLEBOARD = "DRIVE";
  }

  public static class ArmConstants {
    public static class PIDConstants {
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0;
    }
    public static class ArmAngles {
      public static double Stowed = 80;
      public static double L1     = 60;
      public static double L2     = 40;
      public static double L3     = 30;
      public static double L4     = 20;
    }
    public static class CANIDs{
      public static int frontMotorCANID = 14;
      public static int backMotorCANID = 13;
      public static int canandmagCANID = 21;
    }
    public static class Buttons{
      public static int gyroButton = 6;
      public static int stowButton = 5;
      public static int L1Button = 1;
      public static int L2Button = 2;
      public static int L3Button = 3;
      public static int L4Button = 4;
    }
    public static double precisionInDegrees = 1;
  }

  // max speed of the robot in m/s use Units.feetToMeters to use feet
  public static final   double  MAX_SPEED = 4.5;
}







// ------------     CANID Reference     ------------

//  Drive:
//    Krakens:  2(FL), 4(FR), 6(BL), 8(BR)
//    SparkMax: 1(FL), 3(FR), 5(BL), 7(BR
//    Gyro: 20

//  Climb: 
//    SparkMax: 23 (Should Change to 11)

//  Arm:
//    Krakens: 14(F), 13(B)
//    Encoder: 21

// Intake:
//    Kraken: 14

//  Wrist:
//    SparkMax: 15(Rotation), 16(Diff 1), 17(Diff 2)

// RIO: 0
// PDH 10



// FREE CANIDS: 9, 18-19, 21+
