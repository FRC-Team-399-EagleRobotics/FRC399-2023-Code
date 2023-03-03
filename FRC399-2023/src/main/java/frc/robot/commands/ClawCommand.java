package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
 @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

 private final ClawSubsystem m_claw;

 public ClawCommand(ClawSubsystem m_claw) {
    this.m_claw = m_claw;
    addRequirements(m_claw);
 }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
  // Change button to the correct one.
  if (RobotContainer.m_driver.getRawButton(1)){
    m_claw.grab();
  }
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
  m_claw.endGrab();
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
