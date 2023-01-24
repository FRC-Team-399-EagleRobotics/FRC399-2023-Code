package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AutonomousDrive extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DrivetrainSubsystem m_adrive;
    private double l, r, t;
    Timer timer = new Timer();
    Limelight limelight;
  
    boolean isFinished = false;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem aused by this command.
     */
   public AutonomousDrive(DrivetrainSubsystem m_adrive, double l, double r, double t) {
      this.m_adrive = m_adrive;
      this.limelight = new Limelight();
      this.l = l;
      this.r = r;
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

      double pX = 0.02, pY = 0.04;

      double x = limelight.getX() * pX;
      double y = limelight.getY() * pY;

      
      double leftOut = x-y;
      double rightOut = -x-y;

      if (timer.get() < t){
        m_adrive.setTank(leftOut, rightOut);

        
        //m_adrive.setTank(l, r);
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
