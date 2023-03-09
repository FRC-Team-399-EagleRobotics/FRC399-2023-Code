// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GrippyCommand;
import frc.robot.constants.swerveConstants;
import frc.robot.constants.swerveConstants.AutoConstants;
import frc.robot.constants.swerveConstants.DriveConstants;
import frc.robot.constants.swerveConstants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.commands.SwerveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import java.util.List;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
  public static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SwerveCommand m_swerve = new SwerveCommand(m_robotDrive);
  private static SwerveAutoBuilder swerveAutoBuilder; //static for accesibility reasons

  // NavX(gyro)
  private final BalanceCommand m_balance = new BalanceCommand(m_robotDrive);


  // Claw 
  private final GripperSubsystem m_claw = new GripperSubsystem();
  private final GrippyCommand m_clawCommand = new GrippyCommand(m_claw);


  // The driver's controller
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
  /*HashMap<String, Command> eventMap = new HashMap<>(); //TODO: IMPLEMENT FOR DIFFERENT AUTONS
eventMap.put("marker1", new PrintCommand("Passed marker 1"));*/

   ArrayList<PathPlannerTrajectory> straight = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Holonomic Straight", new PathConstraints(0.5, 0.5));

   public static Command buildAuto1(List<PathPlannerTrajectory> trajs) {
  //s_Swerve.resetOdometry(trajs.get(0).getInitialHolonomicPose());
  swerveAutoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose,
      m_robotDrive::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(swerveConstants.AutoConstants.kPXController, 0, 0),
      new PIDConstants(swerveConstants.AutoConstants.kPThetaController, 0, 0),
      m_robotDrive::setModuleStates,
      swerveConstants.AutoConstants.eventMap,
      true,
      m_robotDrive
  );

  return swerveAutoBuilder.fullAuto(trajs);
 // return SwerveAutoBuilder.fullAuto(traj);
   }


  public Command getAutonomousCommand() {
    // Create config for trajectory
/*     TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
*/

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(straight.getInitialHolonomicPose());
    
    // Run path following command, then stop at the end.
    return buildAuto1(straight);
  }
}
