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
    public final static int CAN_LeftFrontSteer = 2;
    public final static int CAN_LeftFrontDrive = 3;
    public final static int CAN_RightFrontSteer = 4;
    public final static int CAN_RightFrontDrive = 5;

    // Order should match side
    public static final int[] turningID = new int[] {CAN_LeftFrontSteer, CAN_LeftBackSteer, CAN_RightFrontSteer, CAN_RightBackSteer};
    public static final int[] spinningID = new int[] {CAN_LeftFrontDrive, CAN_LeftBackDrive, CAN_RightFrontDrive, CAN_RightBackDrive};
    public final static int[] CANCoders = new int[] {CANCoderTopLeft, CANCoderBottomLeft, CANCoderTopRight, CANCoderBottomRight};

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
