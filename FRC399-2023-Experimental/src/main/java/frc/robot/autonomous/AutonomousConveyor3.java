package frc.robot.autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class AutonomousConveyor3 extends CommandBase {
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ConveyorSubsystem m_aconveyor3;
    private double aPwr, bPwr, t;
    private boolean pos;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousConveyor3(ConveyorSubsystem m_aconveyor3, double aPwr, double bPwr, double t) {
        this.m_aconveyor3 = m_aconveyor3;
        this.aPwr = aPwr;
        this.bPwr = bPwr;
        this.t = t;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aconveyor3);
      }

      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
      if (timer.get() < t) {
        m_aconveyor3.setPwr(aPwr, bPwr);
      } else {
        m_aconveyor3.setPwr(0, 0);
        isFinished = true;
      }
      }

      @Override
      public void end(boolean interrupted)
      {
        m_aconveyor3.setPwr(0, 0);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
}
