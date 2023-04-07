package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;

public class GrippyCommand extends CommandBase {
 @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

 private final GripperSubsystem m_grippy;

 public GrippyCommand(GripperSubsystem m_grippy) {
    this.m_grippy = m_grippy;
    addRequirements(m_grippy);
 }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
  // Change button to the correct one.
  if (RobotContainer.m_operator.getRawButton(7)){
    m_grippy.grip();
  } else if (RobotContainer.m_operator.getRawButton(8)){
    m_grippy.reverseGrip();
  } else if (RobotContainer.m_operator.getRawButton(Constants.Controls.rightBumper_ID)){
    m_grippy.slowGrip();
  } else {
    m_grippy.endGrip();
  }
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
  m_grippy.endGrip();
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
