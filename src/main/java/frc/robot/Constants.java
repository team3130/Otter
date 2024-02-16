// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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
  public static final boolean debugMode = true; //TODO: make false after testing
  public static final boolean kNavxReversed = false;
  public final static double kMaxVoltageHopper = 9d;

  public static final int PNM_INTAKE_ACTUATOR = 1;

  public static class CAN {
    // SWERVE CAN NUMBERED LIKE CARTESIAN COORDIANTE QUADRANTS
    // front left

    public final static int PCM = 41;


    public final static int MOD_ONE_STEER = 2;
    public final static int MOD_ONE_DRIVE = 3;

    // back left
    public final static int MOD_TWO_STEER = 4;
    public final static int MOD_TWO_DRIVE = 5;

    // back right
    public final static int MOD_THREE_STEER = 6;
    public final static int MOD_THREE_DRIVE = 7;

    public final static int MOD_FOUR_STEER = 8;
    public final static int MOD_FOUR_DRIVE = 9;

    // front right
    public final static int MOD_ONE_CANCODER = 10;
    public final static int MOD_TWO_CANCODER = 11;
    public final static int MOD_THREE_CANCODER = 12;
    public final static int MOD_FOUR_CANCODER = 13;

    // Order should match side
    public static final int[] turningID = new int[] {MOD_ONE_STEER, MOD_TWO_STEER, MOD_THREE_STEER, MOD_FOUR_STEER};
    public static final int[] spinningID = new int[] {MOD_ONE_DRIVE, MOD_TWO_DRIVE, MOD_THREE_DRIVE, MOD_FOUR_DRIVE};
    public final static int[] CANCoders = new int[] {MOD_ONE_CANCODER, MOD_TWO_CANCODER, MOD_THREE_CANCODER, MOD_FOUR_CANCODER};

    public final static int intakeIndexer = 20;

    public final static int shooterIndexer = 30;
    public final static int shooterTopFlywheel = 31;
    public final static int shooterBottomFlywheel = 32;

    public final static int ampMotor = 40;

    public final static int climberLeft = 20;
    public final static int climberRight = 21;
  }

  public static class IDs {
    public final static int intakeSolenoidOne = 22;
    public final static int intakeSolenoidTwo = 23;


    public final static int intakeLimitSwitch = 21;

    public final static int shooterBreakBeam = 2;

    public final static int ampChannel = 0;
    public final static int ampLimitSwitch = 0;
  }

  public static class AprilTags {
      public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
      // TEST TODO: change vaues obviously
      // Constants such as camera and target height stored. Change per robot and goal!
      public static final double  CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
      // Angle between horizontal and the camera.
      public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
      // TODO: below is wrong
      public static double kTargetPitch = Units.inchesToMeters(0);

      // How far from the target we want to be
      final double GOAL_RANGE_METERS = Units.feetToMeters(3);



      public static final boolean useAprilTags = false;

    // The position and orientation of the camera in meters
    public static final double xPos = Units.inchesToMeters(0);
    public static final double yPos = Units.inchesToMeters(-4);
    public static final double zPos = Units.inchesToMeters(-38);

    // TODO: Find these values
    public static final double pitch = Math.toRadians(-15);
    public static final double yaw = Math.toRadians(0);
    public static final double roll = Math.toRadians(0);

    public static double confidenceN1 = 0; // I'm guessing x component confidence
    public static double confidenceN2 = 0; // I'm guessing y component confidence
    public static double confidenceN3 = 0; // I'm guessing theta component confidence

    public final static int kMedianFilterWindowSize = 9; // median filter size

    public static double kCameraFOV = 0; // TODO: Find real value
    public static final double AprilTagTrustDistance = 5;
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
    // TODO: fix all of these values

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

    public static final boolean kNavxReversed = true; //TODO huh
    public final static double kPhysicalMaxSpeedMetersPerSecond = 4.60; // 3.54 with 8 volts of voltage compensation and 4.19 with 10 volts
    public final static double kMaxSteerVoltage = 5d;
    public final static double kMaxDriveVoltage = 10d;

    public final static double kDeadband = 0.075;

    public final static double kMaxAccelerationDrive = 7.5;
    public final static double kMaxAccelerationAngularDrive = 4.0*Math.PI;

    //TODO TUNE SWERVE
    public final static double kP_FrontRight = 1.35;
    public final static double kI_FrontRight = 0.05;
    public final static double kD_FrontRight = 0;
    public final static double kF_FrontRight = 0;

    public final static double kP_FrontLeft = 1.55;
    public final static double kI_FrontLeft = 0.05;
    public final static double kD_FrontLeft = 0.015;
    public final static double kF_FrontLeft = 0;

    public final static double kP_BackLeft = 1.6;
    public final static double kI_BackLeft = 0.01;
    public final static double kD_BackLeft = 0.015;
    public final static double kF_BackLeft = 0;

    public final static double kP_BackRight = 1.2;
    public final static double kI_BackRight = 0.05;
    public final static double kD_BackRight = 0;
    public final static double kF_BackRight = 0;
    public final static double[] kP_Swerve = new double[] {kP_FrontLeft, kP_BackLeft, kP_FrontRight, kP_BackRight};
    public final static double[] kI_Swerve = new double[] {kI_FrontLeft, kI_BackLeft, kI_FrontRight, kI_BackRight};
    public final static double[] kD_Swerve = new double[] {kD_FrontLeft, kD_BackLeft, kD_FrontRight, kD_BackRight};
    public final static double[] kF_Swerve = new double[] {kF_FrontLeft, kF_BackLeft, kF_FrontRight, kF_BackRight};

    public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
      // TODO: change constants below
      new PIDConstants(3, 0, 0), // Translation PID constants
      new PIDConstants(7, 0, 0), // Rotation PID constants
      3, // Max module speed, in m/s
      0.35, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig() // Default path replanning config. See the API for the options here
    );
  }

  public static class Modules {
    public static final int one = 0;
    public static final int two = 1;
    public static final int three = 2;
    public static final int four = 3;
  }
  
  public static class EncoderOffsets {
    public static final double MOD_ONE_OFFSET = -2.856; // 1.498607; //
    public static final double MOD_TWO_OFFSET = 2.1107; // 1.71; //
    public static final double MOD_THREE_OFFSET = 1.7855; //-0.8436; //
    public static final double MOD_FOUR_OFFSET = 0.56143; //3.14772592; //
    public static final double[] kCANCoderOffsets = new double[] {MOD_ONE_OFFSET, MOD_TWO_OFFSET, MOD_THREE_OFFSET, MOD_FOUR_OFFSET};
  }

  // gear ratios and/or ticks per rev, etc.
  public static class Conversions {
    public final static double kDriveGearRatio = 6.12; // Checked 2/2/24 //6.75  checked 1/19/23
    public final static double kSteerGearRatio = 21.4; // Checked 2/2/24 //150d/7d = 21.42857  checked 1/19
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public final static double DriveRotToMeters = kWheelDiameter * Math.PI * 1/(kDriveGearRatio); // multiply by
    public static final double SteerRotToRads = 1/(kSteerGearRatio) * Math.PI * 2; // multiply by position
    public static final double DriveRotToMetersPerSecond = DriveRotToMeters * 10; // multiply by velocity
    public static final double SteerRotToRadsPerSecond = SteerRotToRads * 10; // multiply by velocity
  }

  public static class Auton {
    /** Chassis auton */
    public static final double kPXController = 3;
    public static final double kIXController = 0;
    public static final double kDXController = 0;
    public static final double kPYController = 3;
    public static final double kIYController = 0;
    public static final double kDYController = 0;
    public static final double kPThetaController = 7;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;

    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI; // max spiny acceleration
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // max spiny velocity
    // real max spiny speed (multiply by some number for safety)
    public static final double kMaxAngularSpeedRadiansPerSecond =  kPhysicalMaxAngularSpeedRadiansPerSecond;
    // spiny PID constraints
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class Buttons {
    // Gamepad Button List
    public static final int LST_BTN_A = 1;
    public static final int LST_BTN_B = 2;
    public static final int LST_BTN_X = 3;
    public static final int LST_BTN_Y = 4;
    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
    public static final int LST_BTN_WINDOW = 7;
    public static final int LST_BTN_MENU = 8;
    public static final int LST_BTN_LJOYSTICKPRESS = 9;
    public static final int LST_BTN_RJOYSTICKPRESS = 10;

    // Gamepad POV List
    public static final int LST_POV_UNPRESSED = -1;
    public static final int LST_POV_N = 0;
    public static final int LST_POV_NE = 45;
    public static final int LST_POV_E = 90;
    public static final int LST_POV_SE = 135;
    public static final int LST_POV_S = 180;
    public static final int LST_POV_SW = 225;
    public static final int LST_POV_W = 270;
    public static final int LST_POV_NW = 315;

    // Gamepad Axis List
    public static final int LST_AXS_LJOYSTICKX = 0;
    public static final int LST_AXS_LJOYSTICKY = 1;
    public static final int LST_AXS_LTRIGGER = 2;
    public static final int LST_AXS_RTRIGGER = 3;
    public static final int LST_AXS_RJOYSTICKX = 4;
    public static final int LST_AXS_RJOYSTICKY = 5;
  }
}
