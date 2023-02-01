package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** An example command that uses an example subsystem. */
public class BalanceCommand extends CommandBase {
 private DrivetrainSubsystem m_tank;
 
 AHRS ahrs;
 boolean autoBalanceXMode;
 boolean autoBalanceYMode;

 double xAxisRate = 0;
 double yAxisRate = 0;

 static final double kOffBalanceAngleThresholdDegrees = 15;
 static final double kOonBalanceAngleThresholdDegrees = 15;

 public BalanceCommand(DrivetrainSubsystem m_tank, AHRS ahrs) {
    this.ahrs = ahrs;
    this.m_tank = m_tank;
    addRequirements(m_tank);
  }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
    try {
        /***********************************************************************
         * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
         * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
         * 
         * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
         * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
         * 
         * VMX-pi: - Communication via USB. - See
         * https://vmx-pi.kauailabs.com/installation/roborio-installation/
         * 
         * Multiple navX-model devices on a single robot are supported.
         ************************************************************************/
        //WPILIB Gyro method
        //gyro = new AHRS(SPI.Port.kMXP);
        //gyro.reset();
        //gyro.calibrate();

        
        // Old NavX method
        ahrs = new AHRS(SPI.Port.kMXP);

        //Extra?
        ahrs.reset();
        ahrs.calibrate();
        


    } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
 } 

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    //Old NavX method
    double pitchAngleDegrees = ahrs.getPitch();
    double rollAngleDegrees = ahrs.getRoll();

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
       m_tank.setTank(yAxisRate, xAxisRate);
     } catch (RuntimeException ex) {
         String err_string = "Drive system error:  " + ex.getMessage();
         DriverStation.reportError(err_string, true);
     }
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   m_tank.setTank(0, 0);
 }
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
 
     
     return false;
   }

}