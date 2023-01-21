// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Controls;
import frc.robot.subsystems.IntakeSubsystem;

import javax.naming.ldap.Control;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem m_intake;
  double stickL = RobotContainer.operator.getRawAxis(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCmd(IntakeSubsystem m_intake, double iPwr, boolean iPos) {
    this.m_intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  public void execute() {
  //if the left trigger is being held, extends and intakes 
    if(RobotContainer.operator.getRawButton(Constants.Controls.leftTrigger_ID)){
          m_intake.intake();
  //if the right trigger is being held, extends and outtakes 
      }else if(RobotContainer.operator.getRawButton(Constants.Controls.leftBumper_ID)){
          m_intake.outTake();
      }else{
          m_intake.endIntake();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.endIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
