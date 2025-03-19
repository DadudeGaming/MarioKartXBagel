// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.dyn4j.geometry.Vector3;

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
      public static final double kP = 0.025;
      public static final double kI = 0;
      public static final double kD = 0;
      // public static final double kMaxVel = 750;
      public static final double kMaxVel = 500;
      // public static final double kMaxAccel = 450;
      public static final double kMaxAccel = 300;
    }
    public static class ArmAngles {
      public static double Stowed = 125;
      public static double L1     = 100;
      public static double L2     = 75;
      public static double L3     = 60;
      public static double L4     = 10;
    }
    public static class CANIDs{
      public static int frontMotorCANID = 12;
      public static int backMotorCANID = 13;
      public static int canandmagCANID = 21;
    }
    public static double precisionInDegrees = 2;
  }

  public static class TelescopeConstants {
    public static class PIDConstants {
      public static final double kP = 0.03;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kMaxVel = 75;
      // public static final double kMaxVel = 50;
      public static final double kMaxAccel = 150;
      // public static final double kMaxAccel = 100;
    }

    public static class TelescopeLengths {
      public static double Stowed = 0;
      public static double L1     = 10;
      public static double L2     = 20;
      public static double L3     = 27;
      public static double L4     = 37;
    }

    public static int MotorCANID = 18;

    public static double precisionInCM = 5;
    
  }


  public static class IntakeConstants {
    public static final double stopCurrent = 50;

    public static final int CANID = 14;
  }
  // max speed of the robot in m/s use Units.feetToMeters to use feet
  public static final   double  MAX_SPEED = 4.5;


  public static final class WristConstants {
      public static final class PID{
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
      }


      public static class CANIDs {
        public static int zMotor = 15;
        public static int yMotor = 16;
        public static int xMotor = 17;
      }

      // Presets for the position of the wrist
      public static final class PRESETS{
        public static Vector3 STOWED = new Vector3(1,2,3);
        public static Vector3 L1 = new Vector3(1,2,3);
        public static Vector3 L2 = new Vector3(1,2,3);
        public static Vector3 L3 = new Vector3(1,2,3);
        public static Vector3 L4 = new Vector3(1,2,3);
        public static Vector3 INTAKE = new Vector3(1,2,3);
      }
    
  }
}


// ------------     CANID Reference     ------------

//  Drive:
//    Krakens:  2(FL), 4(FR), 6(BL), 8(BR)
//    SparkMax: 1(FL), 3(FR), 5(BL), 7(BR
//    Gyro: 20

//  Climb: 
//    SparkMax: 23 (Should Change to 11)

//  Arm:
//    Krakens: 12(F), 13(B)
//    Encoder: 21

// Intake:
//    Kraken: 14

//  Wrist:
//    SparkMax: 15(Rotation), 16(Diff 1), 17(Diff 2)

// Telescope:
//    Kraken: 18


// RIO: 0
// PDH 10



// FREE CANIDS: 9, 19, 21+
