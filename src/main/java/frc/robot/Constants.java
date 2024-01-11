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

  public static class OperatorConstants {

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

}
