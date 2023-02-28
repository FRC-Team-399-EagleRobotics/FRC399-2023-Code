// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousDrive;
import frc.robot.autonomous.AutonomousDrive2;
import frc.robot.autonomous.AutonomousVisionAim;
import frc.robot.commands.Tankdrive;
import frc.robot.commands.VisionAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BalanceCommand;
import frc.robot.Constants.Swerve.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...\

  //public static Joystick driver = new Joystick(1);
  public static Joystick operator = new Joystick(0);

  public static Joystick rightJoy = new Joystick(1);
  public static Joystick leftJoy = new Joystick(2);

  Joystick m_driver = new Joystick(1);

  //public double stickL = RobotContainer.driver.getRawAxis(1);
  //public double stickR = RobotContainer.driver.getRawAxis(5); 

  //----Drivetrain-----
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Tankdrive m_tankdrive = new Tankdrive(m_drivetrainSubsystem, 0, 0);

  // Swerve Drive
  private final DriveSubsystem m_SwerveDrive = new DriveSubsystem();
  
  // NavX(gyro)
  private final BalanceCommand m_balance = new BalanceCommand(m_drivetrainSubsystem);
  // Auton commands
  private final AutonomousDrive m_autodrive = new AutonomousDrive(m_drivetrainSubsystem, 0, 0, 0);
  private final AutonomousDrive2 m_autodrive2 = new AutonomousDrive2(m_drivetrainSubsystem, 0, 0, 0);

  //-----Climber------ 

  //-----Limelight----
  private final Limelight m_limelightSubsystem = new Limelight();
  //private final AutonomousVisionAim m_avision = new AutonomousVisionAim(m_limelightSubsystem, 0.0);


  Limelight limelight = new Limelight();
  private BooleanSupplier visionBs;
  private Trigger visionButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

        // Configure default commands
        m_SwerveDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_SwerveDrive.drive(
                  -MathUtil.applyDeadband(m_driver.getRawAxis(1), Constants.Swerve.OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driver.getRawAxis(0), Constants.Swerve.OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driver.getRawAxis(2), Constants.Swerve.OIConstants.kDriveDeadband),
                  true, true),
                  m_SwerveDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drivetrainSubsystem.setDefaultCommand(m_tankdrive);

    new JoystickButton(m_driver, Button.kR1.value)
    .whileTrue(new RunCommand(
        () -> m_SwerveDrive.setX(),
        m_SwerveDrive));

    //new RepeatCommand(new VisionAimCommand(m_drivetrainSubsystem, limelight));


    //RepeatCommand vision = new RepeatCommand(new VisionAimCommand(m_drivetrainSubsystem, limelight));
    new JoystickButton(operator, Constants.Controls.X_ID).whileTrue(new VisionAimCommand(m_drivetrainSubsystem, limelight));

    new JoystickButton(operator, Constants.Controls.X_ID).whileTrue(new VisionAimCommand(m_drivetrainSubsystem, limelight));
    new JoystickButton(operator, Constants.Controls.B_ID).whileTrue(new BalanceCommand(m_drivetrainSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_SwerveDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_SwerveDrive::setModuleStates,
        m_SwerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_SwerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_SwerveDrive.drive(0, 0, 0, false, false));
    
  }
}
