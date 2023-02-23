// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
public final class Constants {

    public static class Drivetrain{
        public static final int leftDriveCim1_ID = 1;
        public static final int leftDriveCim2_ID = 2;

        public static final int rightDriveCim1_ID = 3;
        public static final int rightDriveCim2_ID = 4;

        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
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
    public static class Swerve {
        public static final class DriveConstants {
          // Driving Parameters - Note that these are not the maximum capable speeds of
          // the robot, rather the allowed maximum speeds
          public static final double kMaxSpeedMetersPerSecond = 4.8;
          public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
      
          public static final double kDirectionSlewRate = 1.2; // radians per second
          public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
          public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
      
          // Chassis configuration
          public static final double kTrackWidth = Units.inchesToMeters(27);
          // Distance between centers of right and left wheels on robot
          public static final double kWheelBase = Units.inchesToMeters(27);
          // Distance between front and back wheels on robot
          public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
      
          // Angular offsets of the modules relative to the chassis in radians
          public static final double kFrontLeftChassisAngularOffset = Math.PI / 2;
          public static final double kFrontRightChassisAngularOffset = Math.PI;
          public static final double kBackLeftChassisAngularOffset = 2 * Math.PI;
          public static final double kBackRightChassisAngularOffset = -Math.PI / 2;
      
          // SPARK MAX CAN IDs
          public static final int kFrontLeftDrivingCanId = 1;
          public static final int kFrontRightDrivingCanId = 2;
          public static final int kRearRightDrivingCanId = 3;
          public static final int kRearLeftDrivingCanId = 4;
      
          public static final int kFrontLeftTurningCanId = 5;
          public static final int kFrontRightTurningCanId = 6;
          public static final int kRearRightTurningCanId = 7;
          public static final int kRearLeftTurningCanId = 8;
      
          public static final boolean kGyroReversed = false;
        }
      
        public static final class ModuleConstants {
          // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
          // This changes the drive speed of the module (a pinion gear with more teeth will result in a
          // robot that drives faster).
          public static final int kDrivingMotorPinionTeeth = 14;
      
          // Invert the turning encoder, since the output shaft rotates in the opposite direction of
          // the steering motor in the MAXSwerve Module.
          public static final boolean kTurningEncoderInverted = true;
      
          // Calculations required for driving motor conversion factors and feed forward
          public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
          public static final double kWheelDiameterMeters = 0.0762;
          public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
          // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
          public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
          public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
              / kDrivingMotorReduction;
      
          public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
              / kDrivingMotorReduction; // meters
          public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
              / kDrivingMotorReduction) / 60.0; // meters per second
      
          public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
          public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
      
          public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
          public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
      
          public static final double kDrivingP = 0.04;
          public static final double kDrivingI = 0;
          public static final double kDrivingD = 0;
          public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
          public static final double kDrivingMinOutput = -1;
          public static final double kDrivingMaxOutput = 1;
      
          public static final double kTurningP = 1;
          public static final double kTurningI = 0;
          public static final double kTurningD = 0;
          public static final double kTurningFF = 0;
          public static final double kTurningMinOutput = -1;
          public static final double kTurningMaxOutput = 1;
      
          public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
          public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
      
          public static final int kDrivingMotorCurrentLimit = 40; // amps
          public static final int kTurningMotorCurrentLimit = 20; // amps
        }
      
        public static final class OIConstants {
          public static final int kDriverControllerPort = 0;
          public static final double kDriveDeadband = 0.1;
        }
      
        public static final class AutoConstants {
          public static final double kMaxSpeedMetersPerSecond = 3;
          public static final double kMaxAccelerationMetersPerSecondSquared = 3;
          public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
          public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
      
          public static final double kPXController = 1;
          public static final double kPYController = 1;
          public static final double kPThetaController = 1;
      
          // Constraint for the motion profiled robot angle controller
          public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }
      
        public static final class NeoMotorConstants {
          public static final double kFreeSpeedRpm = 5676;
        }
      }
}

