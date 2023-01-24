package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WaitSecondsCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DrivetrainSubsystem m_adrive;
    private double t;
    Timer timer = new Timer();
  
    boolean isFinished = false;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem aused by this command.
     */
   public WaitSecondsCommand(DrivetrainSubsystem m_adrive, double t) {
      this.m_adrive = m_adrive;
      this.t = t;
      //Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_adrive);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      timer.reset();
      timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (timer.get() < t){
        m_adrive.setTank(0.0, 0.0);
      }
      else 
      {
        isFinished = true;
      }
    }

    @Override
    public void end(boolean interrupted)
    {
      m_adrive.setTank(0, 0);
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isFinished;
    }
}
