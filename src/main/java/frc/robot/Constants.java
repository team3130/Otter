// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings("ALL")
public final class Constants {
  public static final boolean debugMode = false;
  public static final boolean pitMode = false; 

  public static class CAN {
    public final static int PCM = 1;
    public final static int ampLiftMotor = 14;
    public final static int ampSpinMotor = 28;
    public final static int climberRight = 15;
    public final static int climberLeft = 16;
    public final static int shooterTopFlywheel = 20;
    public final static int shooterBottomFlywheel = 21;
    public final static int intakeIndexer = 22;
  }

  public static class IDs {
    public static final int intakePNMChannel = 4;
    public final static int intakeLimitDIO = 0;
    public final static int shooterBeamDIO = 1;

    public final static int ampLimitDIO = 4;
    public final static int shawtyShifterChannel = 5;
    public final static int longShifterChannel = 6;

    public static final int kRLimitSwitch = 2;
    public static final int kLLimitSwitch = 3;
  }


  public static class AprilTags {
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(57.5);
      // Constants such as camera and target height stored. Change per robot and goal!
    public static final double  CAMERA_HEIGHT_METERS = Units.inchesToMeters(14.9);
    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(16.55);
    public static double kTargetPitch = Units.inchesToMeters(0);
    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    public static final int speakerTargetRedFiducialID = 4;
    public static final int speakerTargetBlueFiducialID = 7;
    public static final int ampTargetRedFiducialID = 5;
    public static final int ampTargetBlueFiducialID = 6;
  }

  /**
   * For swerve drive
   * translations for the distance to each wheel from the center of the bot.
   * Remember that forward (0 radians) is positive X
   * Check:
   *  right half the bot up half the bot      (0.5, 0.5)
   *  right half the bot down half the bot    (-0.5, 0.5)
   *  left half the bot up half the bot       (0.5, -0.5)
   *  left half the bot down half the bot     (-0.5, -0.5)
   * These look like coordinates to each wheel with the order being:
   *  top right,
   *  bottom right,
   *  top left,
   *  bottom left,
   */
  public static class Swerve {
    public static double maxSteerVoltage = 4d;
    public static double maxDriveVoltage = 10d;

    public static double tuningDesiredVelocity = 2d;



    public static double slot0_kS = 0; // DONT USE KS
    public static double slot0_kV = 0.12; // measured 3/19/24
    public static double slot0_kP = 0.2; // measured 3/14/24
    public static double slot0_kI = 0;
    public static double slot0_kD = 0;


    // SWERVE CAN NUMBERED LIKE CARTESIAN COORDIANTE QUADRANTS
    // front right
    public final static int MOD_ONE_STEER = 2;
    public final static int MOD_ONE_DRIVE = 3;

    // back right
    public final static int MOD_TWO_STEER = 4;
    public final static int MOD_TWO_DRIVE = 5;

    // back left
    public final static int MOD_THREE_STEER = 6;
    public final static int MOD_THREE_DRIVE = 7;

    public final static int MOD_FOUR_STEER = 8;
    public final static int MOD_FOUR_DRIVE = 9;
    // front left
    public final static int MOD_ONE_CANCODER = 10;
    public final static int MOD_TWO_CANCODER = 11;
    public final static int MOD_THREE_CANCODER = 12;
    public final static int MOD_FOUR_CANCODER = 13;

    // Order should match side
    public static final int[] turningID = new int[] {MOD_ONE_STEER, MOD_TWO_STEER, MOD_THREE_STEER, MOD_FOUR_STEER};
    public static final int[] spinningID = new int[] {MOD_ONE_DRIVE, MOD_TWO_DRIVE, MOD_THREE_DRIVE, MOD_FOUR_DRIVE};
    public final static int[] CANCoders = new int[] {MOD_ONE_CANCODER, MOD_TWO_CANCODER, MOD_THREE_CANCODER, MOD_FOUR_CANCODER};

    /* Length and width as measured as distances between center of wheels */
    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
    public static final double trackWidth = 0.584;
    // the front-to-back distance between the drivetrain wheels, should be measured from center to center
    public static final double wheelBase = 0.584;

    public static final Translation2d[] moduleTranslations = {
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // 1 pos neg
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0) // 4, pos pos
    };

    // / 3.54 with 8 volts of voltage compensation and 4.19 with 10 volts
    // 4.8 max speed, 5 acceleration, drops to 9.6
    public final static double kPhysicalMaxSpeedMetersPerSecond = 4.8;
    public final static double kDeadband = 0.055;
    public final static double kMaxAccelerationDrive = 5;
    public final static double kMaxAccelerationAngularDrive = 4.0*Math.PI;

    public final static double kP_FrontRight = 1.35;
    public final static double kI_FrontRight = 0.05;
    public final static double kD_FrontRight = 0;
    public final static double kF_FrontRight = 0;

    public final static double kP_FrontLeft = 1.55;
    public final static double kI_FrontLeft = 0.05;
    public final static double kD_FrontLeft = 0.015;
    public final static double kF_FrontLeft = 0;

    public final static double kP_BackLeft = 1.0;
    public final static double kI_BackLeft = 0;
    public final static double kD_BackLeft = 0;
    public final static double kF_BackLeft = 0;

    public final static double kP_BackRight = 1.2;
    public final static double kI_BackRight = 0.05;
    public final static double kD_BackRight = 0;
    public final static double kF_BackRight = 0;
    public final static double[] kP_Swerve = new double[] {kP_FrontLeft, kP_BackLeft, kP_FrontRight, kP_BackRight};
    public final static double[] kI_Swerve = new double[] {kI_FrontLeft, kI_BackLeft, kI_FrontRight, kI_BackRight};
    public final static double[] kD_Swerve = new double[] {kD_FrontLeft, kD_BackLeft, kD_FrontRight, kD_BackRight};
    public final static double[] kF_Swerve = new double[] {kF_FrontLeft, kF_BackLeft, kF_FrontRight, kF_BackRight};
  }

  public static class SwerveModules {
    public static final int one = 0;
    public static final int two = 1;
    public static final int three = 2;
    public static final int four = 3;
  }
  
  public static class SwerveEncoderOffsets {
    public static final double MOD_ONE_OFFSET = -2.856; // 1.498607; //
    public static final double MOD_TWO_OFFSET = 2.1107; // 1.71; //
    public static final double MOD_THREE_OFFSET = 1.7855; //-0.8436; //
    public static final double MOD_FOUR_OFFSET = 0.56143; //3.14772592; //
    public static final double[] kCANCoderOffsets = new double[] {MOD_ONE_OFFSET, MOD_TWO_OFFSET, MOD_THREE_OFFSET, MOD_FOUR_OFFSET};
  }

  // gear ratios and/or ticks per rev, etc.
  public static class SwerveConversions {
    public final static double driveGearRatio = 6.12; // Checked 2/2/24 //6.75  checked 1/19/23
    public final static double steerGearRatio = 21.42857; // Checked 2/2/24 //150d/7d = 21.42857  checked 1/19
    public static final double wheelDiameter = Units.inchesToMeters(3.9);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public final static double driveRotToMeters = wheelDiameter * Math.PI * (1/(driveGearRatio)); // multiply by
    public static final double steerRotToRads = 1/(steerGearRatio) * Math.PI * 2; // multiply by position
    public static final double driveRotToMetersPerSecond = driveRotToMeters * 10; // multiply by velocity
    public static final double steerRotToRadsPerSecond = steerRotToRads * 10; // multiply by velocity
  }

  public static class Auton {
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI; // max spiny acceleration
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // max spiny velocity
    // real max spiny speed (multiply by some number for safety)
    public static final double kMaxAngularSpeedRadiansPerSecond =  kPhysicalMaxAngularSpeedRadiansPerSecond;
    // spiny PID constraints
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  // for AddressableLED adjusted HSV Class
  public static class LEDColors {
    public static final int redHSV = 0;
    public static final int orangeHSV = 3;
    public static final int yellowHSV = 10; // team default yellow
    public static final int lightGreenHSV = 45;
    public static final int darkGreenHSV = 60;
    public static final int tealHSV = 65;
    public static final int lightBlueHSV = 75;
    public static final int blueHSV = 100;
    public static final int darkBlueHSV = 120;
    public static final int lightPurpleHSV = 140;
    public static final int purpleHSV = 150;
    public static final int pinkHSV = 170;
    public static final int hotPink = 179;
  }

  public static class PS5 {
    public static final int BTN_SQUARE = 1;
    public static final int BTN_X = 2;
    public static final int BTN_CIRCLE = 3;
    public static final int BTN_TRIANGLE = 4;
    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;

    public static final int BTN_LJOYSTICK_PRESS = 11;
    public static final int BTN_RJOYSTICK_PRESS = 12;

    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;

    // Gamepad Axis List
    public static final int AXS_LJOYSTICKX = 0;
    public static final int AXS_LJOYSTICKY = 1;
    public static final int AXS_LTRIGGER = 3;
    public static final int AXS_RTRIGGER = 4;
    public static final int AXS_RJOYSTICK_X = 2;
    public static final int AXS_RJOYSTICK_Y = 5;
  }

  public static class XBox {
    // Gamepad Button List
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;

    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;
    public static final int BTN_WINDOW = 7;
    public static final int BTN_MENU = 8;
    public static final int BTN_LJOYSTICK_PRESS = 9;
    public static final int BTN_RJOYSTICK_PRESS = 10;

    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;

    // Gamepad Axis List
    public static final int AXS_LJOYSTICK_X = 0;
    public static final int AXS_LJOYSTICK_Y = 1;
    public static final int AXS_LTRIGGER = 2;
    public static final int AXS_RTRIGGER = 3;
    public static final int AXS_RJOYSTICK_X = 4;
    public static final int AXS_RJOYSTICK_Y = 5;
  }
  public static final boolean navxReversed = false;
}
