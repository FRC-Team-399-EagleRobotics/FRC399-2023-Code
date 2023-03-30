package frc.robot.commands;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.swerveConstants.OIConstants;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class SwerveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_swerve;
      /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveCommand(DriveSubsystem m_swerve) {
    this.m_swerve = m_swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  
  @Override
  public void initialize() {}

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
  @Override
  public void execute() {

    m_swerve.drive(
                -MathUtil.applyDeadband(RobotContainer.m_driver.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driver.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(RobotContainer.m_driver.getRawAxis(2), OIConstants.kDriveDeadband),
                true, true);

    if (RobotContainer.m_driver.getRawButton(Button.kR1.value)){
      m_swerve.setX();
    }

  }
        


  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
      return false;
  }

}
