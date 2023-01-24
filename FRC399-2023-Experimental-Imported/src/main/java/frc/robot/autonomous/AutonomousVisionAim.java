package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AutonomousVisionAim extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Limelight m_avision;
    private DrivetrainSubsystem m_adrive2;
    private double t;
    Timer timer = new Timer();
  
    boolean isFinished = false;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem aused by this command.
     */
   public AutonomousVisionAim(Limelight m_avision, double t) {
      this.m_avision = m_avision;
      this.m_adrive2 = m_adrive2;
      this.t = t;
      addRequirements(m_adrive2);
      //Use addRequirements() here to declare subsystem dependencies.

    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_avision.setLight(true);
      timer.reset();
      timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (timer.get() < t){
        double pX = 0.01, pY = 0.04;

        double x = m_avision.getX() * pX;
        double y = m_avision.getY() * pY;
    
        
        double leftOut = x-y;
        double rightOut = -x-y;

        m_adrive2.setTank(leftOut, rightOut);
    
      }
      else 
      {
        isFinished = true;
      }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_avision.setLight(false);
        m_adrive2.setTank(0, 0);
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isFinished;
    }
}
