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

    public static class Drivetrain{
        public static final int leftDriveCim1_ID = 1;
        public static final int leftDriveCim2_ID = 2;
        public static final int leftDriveFalcon_ID = 3;

        public static final int rightDriveCim1_ID = 4;
        public static final int rightDriveCim2_ID = 5;
        public static final int rightDriveFalcon_ID = 6;
        
        public static final int LeftDriveCim1EnC_A = 0;
        public static final int LeftDriveCim1EnC_B = 1;
        public static final int LeftDriveCim2EnC_A = 2;
        public static final int LeftDriveCim2EnC_B = 3;
        public static final int LeftDriveFalconEnC_A = 4;
        public static final int LeftDriveFalconEnC_B = 5;
            
        public static final int RightDriveCim1EnC_A = 6;
        public static final int RightDriveCim1EnC_B = 7;
        public static final int RightDriveCim2EnC_A = 8;
        public static final int RightDriveCim2EnC_B = 9;
        public static final int RightDriveFalconEnC_A = 10;
        public static final int RightDriveFalconEnC_B = 11;

        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static class Intake {
        public static final int intakeMotor_ID = 7;
        public static final int intakeSolenoid_ID = 7;
    }

    public static class Shooter {
        public static final int shooterL_ID = 13;
        public static final int shooterR_ID = 14;
        public static final int hoodSolenoid_ID = 6;
    }

    public static class Conveyor {
        public static final int topConveyor_ID = 16;
        public static final int bottomConveyor_ID = 15;
    }

    public static class Climber {
        public static final int leftClimberCim1_ID = 9;
        public static final int rightClimberCim1_ID = 10;
        public static final int climberSolenoid_ID = 5;
    }

    public static class Controls {
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
}
