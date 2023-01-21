package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class AutonomousIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private IntakeSubsystem m_aintake;
    private double iPwr, t;
    private boolean iPos, out;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousIntake(IntakeSubsystem m_aintake, double iPwr, boolean iPos, double t) {
        this.m_aintake = m_aintake;
        this.iPwr = iPwr;
        this.iPos = iPos;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aintake);
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
            m_aintake.extend();
            m_aintake.setPwr(iPwr);}
        }else{
            m_aintake.setPwr(0);
            m_aintake.retract();
            isFinished = true;
        }
      }
      @Override
      public void end(boolean interrupted)
      {
        m_aintake.retract();
        m_aintake.setPwr(0);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
      
}
