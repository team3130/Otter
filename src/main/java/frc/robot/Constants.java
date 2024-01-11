// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    public static final Translation2d[] moduleTranslations = {
            new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
            new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
            new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
            new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
    };

    public static final boolean kNavxReversed = true;


    public final static double kP_FrontRight = 1.35;
    public final static double kI_FrontRight = 0.05;
    public final static double kD_FrontRight = 0;
    public final static double kF_FrontRight = 0;

    public final static double kP_FrontLeft = 1.55;
    public final static double kI_FrontLeft = 0.05;
    public final static double kD_FrontLeft = 0.015;
    public final static double kF_FrontLeft = 0;

    public final static double SwerveKpBackLeft = 1.6;
    public final static double SwerveKiBackLeft = 0.01;
    public final static double SwerveKdBackLeft = 0.015;
    public final static double SwerveKfBackLeft = 0;

    public final static double SwerveKpBackRight = 1.2;
    public final static double SwerveKiBackRight = 0.05;
    public final static double SwerveKdBackRight = 0;
    public final static double SwerveKfBackRight = 0;

    public final static double[] SwerveKp = new double[] {kP_FrontLeft, SwerveKpBackLeft, kP_FrontRight, SwerveKpBackRight};
    public final static double[] SwerveKi = new double[] {kI_FrontLeft, SwerveKiBackLeft, kI_FrontRight, SwerveKiBackRight};
    public final static double[] SwerveKd = new double[] {kD_FrontLeft, SwerveKdBackLeft, kD_FrontRight, SwerveKdBackRight};
    public final static double[] SwerveKf = new double[] {kF_FrontLeft, SwerveKfBackLeft, kF_FrontRight, SwerveKfBackRight};
  }

  public static class EncoderOffsets {
    public static final double kTopLeftOffset = Math.toRadians(268.682);
    public static final double kBottomLeftOffset = Math.toRadians(281.426);
    public static final double kTopRightOffset = Math.toRadians(129.3);
    public static final double kBottomRightOffset = Math.toRadians(0);
    public static final double[] kCANCoderOffsets = new double[] {kTopLeftOffset, kBottomLeftOffset, kTopRightOffset, kBottomRightOffset};

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

}
