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
    


  public static class Gripper {
      // Change name/ID to corresponding. This is a template

      public static final int gripperMotor1_ID = 11;
      public static final int gripperMotor2_ID = 12;
  
  }

  public static class Arm {
      // Change name/ID to corresponding. This is a template


      public static final int wristMotor = 13;

      //public static final int wristSolenoid_ID = 2;
      public static final int clawSolenoid_ID = 1;
      public static final int extensionSolenoid_ID = 0;

      //timeout for motion magic
      public static final int kTimeoutMs = 30;
  }

  public static class Controls{
      public static final int driver = 0;
      public static final int operator = 1;

      public static final int X_ID = 1;
      public static final int A_ID = 2;
      public static final int B_ID = 3;
      public static final int Y_ID = 4;
      public static final int leftBumper_ID = 5;
      public static final int rightBumper_ID = 6;
      public static final int leftTrigger_ID = 7;
      public static final int rightTrigger_ID = 8;
      public static final int start_ID = 10;
  }
