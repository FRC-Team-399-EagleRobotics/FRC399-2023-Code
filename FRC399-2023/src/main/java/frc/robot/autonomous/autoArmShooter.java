package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase; 

public class autoArmShooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ArmSubsystem m_aArmS;
    private double t;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public autoArmShooter(ArmSubsystem m_aArmS, double t) {
        this.m_aArmS = m_aArmS;

        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aArmS);
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
            m_aArmS.cubeMid();
        }
        }else{
            m_aArmS.cubeMid();
            isFinished = true;
        }
      }
      @Override
      public void end(boolean interrupted)
      {
        m_aArmS.cubeMid();
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
    }
