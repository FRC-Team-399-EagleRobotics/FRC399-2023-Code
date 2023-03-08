// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GrippyCommand;
import frc.robot.constants.swerveConstants.AutoConstants;
import frc.robot.constants.swerveConstants.DriveConstants;
import frc.robot.constants.swerveConstants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.commands.SwerveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Arm 
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final ArmCommand m_armCommand = new ArmCommand(m_ArmSubsystem);

  // The Swerve subsystem
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final SwerveCommand m_swerve = new SwerveCommand(m_robotDrive);

  // NavX(gyro)
  private final BalanceCommand m_balance = new BalanceCommand(m_robotDrive);

  // Claw 
  private final GripperSubsystem m_claw = new GripperSubsystem();

  private final GrippyCommand m_clawCommand = new GrippyCommand(m_claw);

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //PS4Controller m_driver = new PS4Controller(OIConstants.kDriverControllerPort);
  public static Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  public static Joystick m_operator = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(m_swerve);
        //m_robotDrive.setDefaultCommand(m_armCommand);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_robotDrive.setDefaultCommand(m_balance);
    m_claw.setDefaultCommand(m_clawCommand);
    m_ArmSubsystem.setDefaultCommand(m_armCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.

    // THIS DOES NOT WORK PROPERLY
    Trajectory backwards = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(1, 1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-3, 0, new Rotation2d(0)),
        config);

    PathPlannerTrajectory straightPath = PathPlanner.loadPath("Straight Path", new PathConstraints(.5, .25));
    PathPlannerTrajectory spin3 = PathPlanner.loadPath("Spin3", new PathConstraints(1, .5));
    PathPlannerTrajectory straightPathReverse = PathPlanner.loadPath("Straight Path(reverse)", new PathConstraints(1, .5));

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        straightPathReverse,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0.0, 0),
        new PIDController(AutoConstants.kPYController, 0.0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
      

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straightPathReverse.getInitialPose());
    //m_robotDrive.resetOdometry(backwards.getInitialPose());
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
