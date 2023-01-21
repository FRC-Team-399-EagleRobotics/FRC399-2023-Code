// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCmd extends CommandBase {
  /** Creates a new Climber. */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ClimberSubsystem m_climber;

  public ClimberCmd(ClimberSubsystem m_climber, double cPWR) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climber = m_climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if(RobotContainer.operator.getPOV() == 0) {
    m_climber.up();
  }else if(RobotContainer.operator.getPOV() == 180){
    m_climber.down();
  }else{
    m_climber.endMotors();
  }

  if(RobotContainer.operator.getRawButton(Constants.Controls.start_ID)){
    m_climber.active();
  }else{
    m_climber.inactive();
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}