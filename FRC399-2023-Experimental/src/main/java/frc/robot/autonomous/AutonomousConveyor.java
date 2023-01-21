package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class AutonomousConveyor extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ConveyorSubsystem m_aconveyor;
    private double aPwr, bPwr, t;
    private boolean pos;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousConveyor(ConveyorSubsystem m_aconveyor, double aPwr, double bPwr, double t) {
        this.m_aconveyor = m_aconveyor;
        this.aPwr = aPwr;
        this.bPwr = bPwr;
        this.t = t;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aconveyor);
      }

      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
      if (timer.get() < t) {
        m_aconveyor.setPwr(aPwr, bPwr);
      } else {
        m_aconveyor.setPwr(0, 0);
        isFinished = true;
      }
      }

      @Override
      public void end(boolean interrupted)
      {
        m_aconveyor.setPwr(0, 0);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
}
