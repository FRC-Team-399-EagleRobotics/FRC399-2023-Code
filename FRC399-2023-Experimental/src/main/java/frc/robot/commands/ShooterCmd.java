package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class ShooterCmd extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ShooterSubsystem m_shooter;

    public ShooterCmd(ShooterSubsystem m_shooter, double vel, boolean pos) {
        this.m_shooter = m_shooter;
        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_shooter);
      }

    @Override
    public void initialize() {

    }

  @Override
  public void execute() {
    if(RobotContainer.operator.getRawButton(Constants.Controls.B_ID)){
        m_shooter.highShot();
    }else if(RobotContainer.operator.getRawButton(Constants.Controls.A_ID)){
        m_shooter.lowShot();
    }else if(RobotContainer.operator.getRawButton(Constants.Controls.X_ID)){
        m_shooter.funnyShot();
    }else{
        m_shooter.endShooter();
    }
}

    @Override
    public void end(boolean interrupted) {
        m_shooter.endShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
