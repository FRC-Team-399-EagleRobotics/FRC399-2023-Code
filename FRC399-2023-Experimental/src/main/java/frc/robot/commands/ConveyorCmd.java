package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.Controls;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorCmd extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ConveyorSubsystem m_conveyor;
    
      public ConveyorCmd(ConveyorSubsystem m_conveyor, double aPwr, double bPwr) {
        this.m_conveyor = m_conveyor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_conveyor);
      }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Pressing Right trigger to store.
        if (RobotContainer.operator.getRawButton(Controls.leftTrigger_ID)){
            m_conveyor.intake();
        // Pressing Right bumper leads to spitting out ball
        } else if (RobotContainer.operator.getRawButton(Controls.leftBumper_ID)) {
            m_conveyor.spit();
        // Press Left Trigger to shoot. Waiting for shooter for the meantime just going to run conveyor
        } else if (RobotContainer.operator.getRawButton(Controls.rightTrigger_ID)) {
            m_conveyor.load();
        }else if(RobotContainer.operator.getRawButton(Controls.rightBumper_ID)){
            m_conveyor.store();

        }
        else {
            m_conveyor.endConveyor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.endConveyor();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

 
}
