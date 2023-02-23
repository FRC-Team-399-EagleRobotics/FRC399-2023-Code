package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class BalanceCommandSwerve extends CommandBase {
 private DriveSubsystem m_swerve;

 boolean autoBalanceXMode;
 boolean autoBalanceYMode;

 double xAxisRate = 0;
 double yAxisRate = 0;

 static final double kOffBalanceAngleThresholdDegrees = 15;
 static final double kOonBalanceAngleThresholdDegrees = 15;

 public BalanceCommandSwerve(DriveSubsystem m_swerve) {
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);
  }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
  m_swerve.resetGyro();
 } 

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
  // Gets NavX variables from drivetrain subsystem
   double pitchAngleDegrees = m_swerve.getPitch();
   double rollAngleDegrees = m_swerve.getRoll();
   
    if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees)||(Math.abs(pitchAngleDegrees)<=Math.abs(kOffBalanceAngleThresholdDegrees*-1)))) {
         autoBalanceXMode = true;
     } else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees)&&(Math.abs(pitchAngleDegrees)>=Math.abs(kOffBalanceAngleThresholdDegrees*-1)))) {
         autoBalanceXMode = false;
     }
     if (!autoBalanceYMode && (Math.abs(rollAngleDegrees) >= Math.abs(kOonBalanceAngleThresholdDegrees)||(Math.abs(pitchAngleDegrees)<=Math.abs(kOonBalanceAngleThresholdDegrees*-1)))) {
         autoBalanceYMode = true;
     } else if (autoBalanceYMode && (Math.abs(rollAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))&&(Math.abs(pitchAngleDegrees)>=Math.abs(kOonBalanceAngleThresholdDegrees*-1))) {
         autoBalanceYMode = false;
     }

     // Control drive system automatically,
     // driving in reverse direction of pitch/roll angle,
     // with a magnitude based upon the angle

    if (autoBalanceXMode) {
         double pitchAngleRadians = (pitchAngleDegrees) * (Math.PI / 180.0)*2;
         xAxisRate = Math.sin(pitchAngleRadians) * -1;
     }
     if (autoBalanceYMode) {
         double rollAngleRadians = (rollAngleDegrees) * (Math.PI / 180.0)*2;
         yAxisRate = Math.sin(rollAngleRadians) * -1;
     }

    //Run drivetrain
     try {
         //Arcade drive with a given forward and turn rate */
         m_swerve.drive(yAxisRate, yAxisRate);
     } catch (RuntimeException ex) {
         String err_string = "Drive system error:  " + ex.getMessage();
         DriverStation.reportError(err_string, true);
     }
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
  m_swerve.setTank(0, 0);
 }
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
 
     
     return false;
   }

}