// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static class CAN {
    public final static int leftFrontSteer = 2;
    public final static int leftFrontDrive = 3;
    public final static int rightFrontSteer = 4;
    public final static int rightFrontDrive = 5;

    public final static int leftBackSteer = 6;
    public final static int leftBackDrive = 7;
    public final static int rightBackSteer = 8;
    public final static int rightBackDrive = 9;

    public final static int CANCoderTopRight = 10;
    public final static int CANCoderBottomRight = 11;
    public final static int CANCoderTopLeft = 12;
    public final static int CANCoderBottomLeft = 13;

    // Order should match side
    public static final int[] turningID = new int[] {leftFrontSteer, leftBackSteer, rightFrontSteer, rightBackSteer};
    public static final int[] spinningID = new int[] {leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive};
    public final static int[] CANCoders = new int[] {CANCoderTopLeft, CANCoderBottomLeft, CANCoderTopRight, CANCoderBottomRight};

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
    public static final double trackWidth = 0.61;
    // the front-to-back distance between the drivetrain wheels, should be measured from center to center
    public static final double wheelBase = 0.61;

    public static final Translation2d[] moduleTranslations = {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    };

    public static final boolean kNavxReversed = true;
    public final static double kPhysicalMaxSpeedMetersPerSecond = 4.19; // 3.54 with 8 volts of voltage compensation and 4.19 with 10 volts

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


  }

  public static class Modules {
    public static final int leftFront = 0;
    public static final int leftBack = 1;
    public static final int rightFront = 2;
    public static final int rightBack = 3;
  }

  public static class EncoderOffsets {
    public static final double kTopLeftOffset = Math.toRadians(268.682);
    public static final double kBottomLeftOffset = Math.toRadians(281.426);
    public static final double kTopRightOffset = Math.toRadians(129.3);
    public static final double kBottomRightOffset = Math.toRadians(0);
    public static final double[] kCANCoderOffsets = new double[] {kTopLeftOffset, kBottomLeftOffset, kTopRightOffset, kBottomRightOffset};

  }

  // gear ratios and/or ticks per rev, etc.
  public static class Conversions {
    public final static double kDriveGearRatio = 6.75; // checked 1/19
    public final static double kSteerGearRatio = 150d/7d; // checked 1/19
    public static final double kEncoderResolution = 2048;
    public static final double kWheelDiameter = Units.inchesToMeters(3.86);
    public final static double DriveTicksToMeters = kWheelDiameter * Math.PI * 1/(kEncoderResolution * kDriveGearRatio); // multiply by
    public static final double SteerTicksToRads = 1/(kEncoderResolution * kSteerGearRatio) * Math.PI * 2; // multiply by position
    public static final double DriveTicksToMetersPerSecond = DriveTicksToMeters * 10; // multiply by velocity
    public static final double SteerTicksToRadsPerSecond = SteerTicksToRads * 10; // multiply by velocity
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

}
