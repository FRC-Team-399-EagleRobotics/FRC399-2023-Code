package frc.robot.commands;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.swerveConstants.OIConstants;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class SwerveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_swerve;

    double x = 0;
    double y = RobotContainer.m_driver.getRawAxis(0);
    double offset = 0;
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
  public void initialize() {

  }

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
  @Override
  public void execute() {
    double pX = 0.02, pY = 0.04;

    // Autodrive will be true if left or right trigger is being pressed or the right bumper.
    boolean left = RobotContainer.m_driver.getRawButton(7);
    boolean center = RobotContainer.m_driver.getRawButton(5); 
    boolean right = RobotContainer.m_driver.getRawButton(6); 

    boolean autoDrive = left|| center || right;
    double steer = 0;
    //System.out.println(m_swerve.getHeading());
    if(autoDrive) {
      // Left, right, and center has different offsets meaning the bot will move a bit away from the apriltag depending of the offset
      // NOT TESTED AND UNSURE!
      if (left) {
        offset = -.02;
      }  else if (center) {
        offset = 0;
      } else if (right) {
        offset = .02;
      }

      // Lock Y to the apriltag. Aka can't limelight will move you left and right to center to goal
      y = m_swerve.getX() * pX + offset;

      // Allow you to move forward and back
      x = RobotContainer.m_driver.getRawAxis(1);
      // Centers
      steer = 0.02 * (AngleDifference(m_swerve.getHeading(), 180));
        if(Math.abs(steer) > .4) {
          steer = .4 * Math.signum(steer);
      }
      // Print info about limelight and motor for left, right, and center offset configuration
      System.out.println("Limelight: " + m_swerve.getX()*pX);
      System.out.println("Driver: " + y);
      //y = m_swerve.getY() * pY;
      
      // Else allow the driver full control to x, y, and steering(z) axis
    } else {
      x = RobotContainer.m_driver.getRawAxis(1)*.4;
      y = RobotContainer.m_driver.getRawAxis(0)*.4;


      if(RobotContainer.m_driver.getRawButton(1)) {
        steer = 0.02 * (AngleDifference(m_swerve.getHeading(), -90));
        if(Math.abs(steer) > .4) {
          steer = .4 * Math.signum(steer);
        }

      } else if(RobotContainer.m_driver.getRawButton(2)) {
        steer = 0.02 * (AngleDifference(m_swerve.getHeading(), 180));
        if(Math.abs(steer) > .4) {
          steer = .4 * Math.signum(steer);
        }

      } else if(RobotContainer.m_driver.getRawButton(3)) {
        steer = 0.02 * (AngleDifference(m_swerve.getHeading(), 90));
        if(Math.abs(steer) > .4) {
          steer = .4 * Math.signum(steer);
        }

      }else {
        steer = RobotContainer.m_driver.getRawAxis(2);
      }
    }

    m_swerve.drive(
                -MathUtil.applyDeadband(x, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(y, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(steer, OIConstants.kDriveDeadband),
                true, true);

    if (RobotContainer.m_driver.getRawButton(Button.kR1.value)){
      m_swerve.setX();
    }

  }

  public static double AngleDifference(double desiredAngle, double currentAngle) {
    double difference = desiredAngle - currentAngle;

    difference = difference % 360;
    
    return difference >  90 ? 180 - difference : difference;
}
      

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
      return false;
  }

}
