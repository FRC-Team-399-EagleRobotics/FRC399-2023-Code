// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem m_arm) {
    this.m_arm = m_arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time th scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.m_operator.getPOV() == 0) {
      m_arm.high();
    } else if (RobotContainer.m_operator.getPOV() == 180){
      m_arm.mid();
    } else if (RobotContainer.m_operator.getRawButton(Constants.Controls.rightBumper_ID)) {
      m_arm.highIntake();
    } else if (RobotContainer.m_operator.getRawButton(Constants.Controls.leftBumper_ID)){
      m_arm.lowIntake();
    } else if (RobotContainer.m_operator.getRawButton(Constants.Controls.A_ID)){
      m_arm.cubeShooter();
    } else if (RobotContainer.m_operator.getRawButton(Constants.Controls.B_ID)){
      m_arm.stow();
    } /*else if ((RobotContainer.m_operator.getRawAxis(1) > 0) ||  (RobotContainer.m_operator.getRawAxis(1) < 0)) {
      m_arm.manual(RobotContainer.m_operator.getRawAxis(1));
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
