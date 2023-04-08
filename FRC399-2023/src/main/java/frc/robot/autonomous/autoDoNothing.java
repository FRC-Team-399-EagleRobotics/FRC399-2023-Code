package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants.Gripper;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class autoDoNothing extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private GripperSubsystem m_aGrip;
    private double iPwr, t;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public autoDoNothing(GripperSubsystem m_aGrip, double t) {
        this.m_aGrip = m_aGrip;
        this.iPwr = iPwr;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aGrip);
      }  
      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() { // TODO Don't need true or false out take and intake
        if (timer.get() < t) {
        {
            m_aGrip.endGrip();
        }
        }else{
            m_aGrip.endGrip();
            isFinished = true;
        }
      }
      @Override
      public void end(boolean interrupted)
      {
        m_aGrip.endGrip();
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
      
}