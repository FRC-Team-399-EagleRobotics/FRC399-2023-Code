package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
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
    m_claw.exampleCondition();
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    m_claw.exampleCondition();
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {

 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
