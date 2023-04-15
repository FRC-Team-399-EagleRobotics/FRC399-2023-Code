package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class autoSetX extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DriveSubsystem m_drive;
    private double iPwr, t;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public autoSetX(DriveSubsystem m_drive, double t) {
        this.m_drive = m_drive;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
      }  
      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() { // TODO Don't need true or false out take and intake
        
            m_drive.setX();
        
      }
      @Override
      public void end(boolean interrupted)
      {
    
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
      
}