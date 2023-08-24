package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class autoArmIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ArmSubsystem m_aArmM;
    private double t;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public autoArmIntake(ArmSubsystem m_aArmM, double t) {
        this.m_aArmM = m_aArmM;

        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aArmM);
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
            m_aArmM.cubeLowIntake();
        }
        }else{
            m_aArmM.cubeLowIntake();
            isFinished = true;
        }
      }
      @Override
      public void end(boolean interrupted)
      {
        m_aArmM.stow();
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
      
}