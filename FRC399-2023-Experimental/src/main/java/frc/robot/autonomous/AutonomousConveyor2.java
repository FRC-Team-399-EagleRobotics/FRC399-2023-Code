package frc.robot.autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class AutonomousConveyor2 extends CommandBase {
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ConveyorSubsystem m_aconveyor2;
    private double aPwr, bPwr, t;
    private boolean pos;
    Timer timer = new Timer();
    
    boolean isFinished = false;

    public AutonomousConveyor2(ConveyorSubsystem m_aconveyor2, double aPwr, double bPwr, double t) {
        this.m_aconveyor2 = m_aconveyor2;
        this.aPwr = aPwr;
        this.bPwr = bPwr;
        this.t = t;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_aconveyor2);
      }

      @Override
      public void initialize() {
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
      if (timer.get() < t) {
        m_aconveyor2.setPwr(aPwr, bPwr);
      } else {
        m_aconveyor2.setPwr(0, 0);
        isFinished = true;
      }
      }

      @Override
      public void end(boolean interrupted)
      {
        m_aconveyor2.setPwr(0, 0);
      }

      @Override
      public boolean isFinished() {
        return isFinished;
      }
}
