package frc.robot.autonomous;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.commands.ConveyorCmd;

public class AutonomousShooter2 extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ShooterSubsystem m_ashooter2;
    private double vel, t;
    private boolean pos;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousShooter2(ShooterSubsystem m_ashooter2, double vel, boolean pos, double t) {
        this.m_ashooter2 = m_ashooter2;
        this.vel = vel;
        this.pos = pos;
        this.t = t;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_ashooter2);
      }
      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
        if (timer.get() < t) {
            //m_ashooter.setVel(vel);
            m_ashooter2.funnyShot();
      }else {
            m_ashooter2.setVel(0);
            isFinished = true;
      }
        }
      @Override
      public void end(boolean interrupted)
      {
        m_ashooter2.setVel(0);
        m_ashooter2.setHood(true);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
}
