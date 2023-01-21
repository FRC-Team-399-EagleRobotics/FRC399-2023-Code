// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.AutonomousDrive;
import frc.robot.autonomous.AutonomousDrive2;
import frc.robot.autonomous.AutonomousVisionAim;
import frc.robot.commands.Tankdrive;
import frc.robot.commands.VisionAimCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;


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
  private Button visionButton;


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


    new JoystickButton(rightJoy, 11)
    .or(new JoystickButton(rightJoy, 12))
      .or(new JoystickButton(rightJoy, 13))
        .or(new JoystickButton(rightJoy, 14))
          .or(new JoystickButton(rightJoy, 15))
          .or(new JoystickButton(rightJoy, 5))
          .or(new JoystickButton(rightJoy, 6))
          .or(new JoystickButton(rightJoy, 7))
          .or(new JoystickButton(rightJoy, 8))
          .or(new JoystickButton(rightJoy, 9))
          .or(new JoystickButton(rightJoy, 10))
        .whileActiveContinuous(new VisionAimCommand(m_drivetrainSubsystem, limelight));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {

    // CommandGroup auton = new CommandGroup();
    
    // Drive code
    //AutonomousDrive forward = new AutonomousDrive(m_drivetrainSubsystem, 0.24, 0.25, 4);
    //AutonomousDrive spin = new AutonomousDrive(m_drivetrainSubsystem, 1, -1, 15);
    //WaitSecondsCommand wait = new WaitSecondsCommand(m_drivetrainSubsystem, 5.0); // Wait for 10 secs


    //AutonomousConveyor autoConveyor = new AutonomousConveyor(m_conveyorSubsystem, 0.8, -0.8, 3);

    //move forward to line up for shot 
    //AutonomousShooter autoShooter = new AutonomousShooter(m_shooterSubsystem, 0.75, true, 3); // shoots without 
    //AutonomousVisionAim autoVision = new AutonomousVisionAim(m_avision, 4);
    //AutonomousShooter autoLowShooter = new AutonomousShooter(m_shooterSubsystem, 0.4, false, 3);
    
    
    //ParallelCommandGroup grabBall = new ParallelCommandGroup(reverse, autoIntake, autoConveyor); // Bundles commands to run at the same time
    //ParallelCommandGroup stageHigh = new ParallelCommandGroup(autoShooter, autoConveyor2); 
    //ParallelCommandGroup stageLow = new ParallelCommandGroup(autoLowShooter, forward, autoConveyor2);
    //ParallelCommandGroup stageLowSingle = new ParallelCommandGroup(autoLowShooter, autoConveyor2);

    //SequentialCommandGroup visionaim = new SequentialCommandGroup(

    //SequentialCommandGroup autonHigh = new SequentialCommandGroup(grabBall, stageHigh, autoConveyor3); // Runs each command in order //
    //SequentialCommandGroup autonLow = new SequentialCommandGroup(grabBall, stageLow, autoConveyor3); // use for low goal shot with 2
    //SequentialCommandGroup autonLowSingle = new SequentialCommandGroup(stageLowSingle, autoConveyor3); // use to just score a single low ball
    //SequentialCommandGroup autonBackUp = new SequentialCommandGroup(reverse); // use when we just want to back up out of the tarmac
    //SequentialCommandGroup venturaSpecial = new SequentialCommandGroup(spin); // Spin2Win

    //Auton single high and intake enemy ball
    AutonomousDrive reverse = new AutonomousDrive(m_drivetrainSubsystem, -0.24, -0.25, 3);
    /*AutonomousDrive2 turn = new AutonomousDrive2(m_drivetrainSubsystem, -0.25, 0.25, 1);
    AutonomousDrive2 forward = new AutonomousDrive2(m_drivetrainSubsystem, -0.24, -0.25, 2);
    AutonomousDrive2 backFast = new AutonomousDrive2(m_drivetrainSubsystem, 0.75, 0.75, 0.5);
    AutonomousIntake autoIntake = new AutonomousIntake(m_intakeSubsystem, -1, true,  2); 
    AutonomousConveyor2 autoConveyor2 = new AutonomousConveyor2(m_conveyorSubsystem, 0.8, -0.8, 3.5);
    AutonomousConveyor3 autoConveyor3 = new AutonomousConveyor3(m_conveyorSubsystem, -1, -1, 1);
    AutonomousShooter autoShooter = new AutonomousShooter(m_shooterSubsystem, 0.7, true, 3.5);
    AutonomousShooter2 autoRevShot = new AutonomousShooter2(m_shooterSubsystem, 0.7, true, 2);
    ParallelCommandGroup stageShot = new ParallelCommandGroup(autoRevShot, autoConveyor3);
    ParallelCommandGroup stageHigh = new ParallelCommandGroup(reverse, autoShooter, autoConveyor2);
    ParallelCommandGroup IntakeBall = new ParallelCommandGroup(forward, autoIntake);*/
    ParallelCommandGroup prank = new ParallelCommandGroup(reverse);
    SequentialCommandGroup drive = new SequentialCommandGroup(prank);
    

    //Auton double high
    /*AutonomousDrive reverse = new AutonomousDrive(m_drivetrainSubsystem, -0.24, -0.25, 3);
    AutonomousIntake autoIntake = new AutonomousIntake(m_intakeSubsystem, -1, true,  2); 
    AutonomousConveyor2 autoConveyor2 = new AutonomousConveyor2(m_conveyorSubsystem, 0.8, -0.8, 3.5);
    AutonomousConveyor3 autoConveyor3 = new AutonomousConveyor3(m_conveyorSubsystem, -1, -1, 1);
    AutonomousShooter autoShooter = new AutonomousShooter(m_shooterSubsystem, 0.7, true, 3.5);
    AutonomousShooter2 autoRevShot = new AutonomousShooter2(m_shooterSubsystem, 0.7, true, 2);
    ParallelCommandGroup stageShot = new ParallelCommandGroup(autoRevShot, autoConveyor3);
    ParallelCommandGroup stageHigh = new ParallelCommandGroup(reverse, autoIntake, autoShooter, autoConveyor2);
    SequentialCommandGroup autonHighDouble = new SequentialCommandGroup(stageHigh, stageShot);*/

    //Auton single high
    /*AutonomousDrive reverse = new AutonomousDrive(m_drivetrainSubsystem, -0.24, -0.25, 3);
    AutonomousConveyor2 autoConveyor2 = new AutonomousConveyor2(m_conveyorSubsystem, 0.8, -0.8, 3.5);
    AutonomousConveyor3 autoConveyor3 = new AutonomousConveyor3(m_conveyorSubsystem, -1, -1, 1);
    AutonomousShooter autoShooter = new AutonomousShooter(m_shooterSubsystem, 0.7, true, 3.5);
    AutonomousShooter2 autoRevShot = new AutonomousShooter2(m_shooterSubsystem, 0.7, true, 2);
    ParallelCommandGroup stageShot = new ParallelCommandGroup(autoRevShot, autoConveyor3);
    ParallelCommandGroup stageHigh = new ParallelCommandGroup(reverse, autoShooter, autoConveyor2);
    SequentialCommandGroup autonHighSingle = new SequentialCommandGroup(stageHigh, stageShot);*/

    
    return drive; // Runs the commands
    
  }
}
