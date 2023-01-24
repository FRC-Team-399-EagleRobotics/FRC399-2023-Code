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
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousDrive;
import frc.robot.autonomous.AutonomousDrive2;
import frc.robot.autonomous.AutonomousVisionAim;
import frc.robot.commands.Tankdrive;
import frc.robot.commands.VisionAimCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...\

  public static Joystick leftJoy = new Joystick(1);
  public static Joystick rightJoy = new Joystick(2);
  public static Joystick operator = new Joystick(0);
  public double stickL = RobotContainer.leftJoy.getRawAxis(1);
  public double stickR = RobotContainer.rightJoy.getRawAxis(1); 

  //----Drivetrain-----
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Tankdrive m_tankdrive = new Tankdrive(m_drivetrainSubsystem, 0, 0);
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

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drivetrainSubsystem.setDefaultCommand(m_tankdrive);

    //new RepeatCommand(new VisionAimCommand(m_drivetrainSubsystem, limelight));


    //RepeatCommand vision = new RepeatCommand(new VisionAimCommand(m_drivetrainSubsystem, limelight));

    new JoystickButton(operator, Constants.Controls.A_ID).whileTrue(new VisionAimCommand(m_drivetrainSubsystem, limelight));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {

    AutonomousDrive reverse = new AutonomousDrive(m_drivetrainSubsystem, -0.24, -0.25, 3);
    //ParallelCommandGroup vision = new ParallelCommandGroup(reverse);
    SequentialCommandGroup drive = new SequentialCommandGroup(reverse);
    

    return drive; // Runs the commands
    
  }
}
