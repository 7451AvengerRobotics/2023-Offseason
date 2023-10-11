// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class PortConstants {
  public static final int[] LEFT_DRIVE = { 1, 4 };
  public static final int[] RIGHT_DRIVE = { 2, 3 };

  
  public static final int VirtualFourBar = 7;
  public static final int[] Claw = {5, 12}; // need to change port numbers
  public static final int Gyro = 8;
  public static final int Turret = 6;
  public static final int[] Arm = {0, 7, 6};
  public static final int[] CLAW_PNEUMATIC = {1, 7};
  //public static final int Claw = 13;
}
