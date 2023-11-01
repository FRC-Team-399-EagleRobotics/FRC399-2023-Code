// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.PositionState;
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


  boolean lastStow = false;
  // Called every time th scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angle = m_arm.getAngle();


    double adjusted = (angle/225) * (3.1415/2);

    double kF = Math.sin(adjusted * -1);

    //m_arm.setKf(kF);
    // Reserve buttons 7 8 9 for limelight
    if (RobotContainer.m_operator.getRawButton(1)) {
      m_arm.setState(PositionState.CONE_HIGH);
    } else if (RobotContainer.m_operator.getRawButton(2)){
      m_arm.setState(PositionState.CUBE_HIGH);
    } else if (RobotContainer.m_operator.getRawButton(3)) {
      m_arm.setState(PositionState.CONE_MID);
    } else if (RobotContainer.m_operator.getRawButton(4)){
      m_arm.setState(PositionState.CUBE_MID);
    } else if (RobotContainer.m_operator.getRawButton(5)){
      m_arm.setState(PositionState.CONE_LOW);
    } else if (RobotContainer.m_operator.getRawButton(6)) {
      m_arm.setState(PositionState.CUBE_LOW);
    } else if (RobotContainer.m_operator.getRawButton(7)) {
      m_arm.setState(PositionState.CONE_FLOOR);
    } else if (RobotContainer.m_operator.getRawButton(8)) {
      m_arm.setState(PositionState.CUBE_FLOOR);
    } else if (RobotContainer.m_operator.getRawButton(9)) {
      m_arm.setState(PositionState.CONE_HP);
      lastStow = true;
    } else if (RobotContainer.m_operator.getRawButton(10)) {
      m_arm.setState(PositionState.CUBE_HP);
      lastStow = true;
    } else {

      if(lastStow) {
        m_arm.setState(PositionState.STOW);
        lastStow = false;
      }

    }
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
