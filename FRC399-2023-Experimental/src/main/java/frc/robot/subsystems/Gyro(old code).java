package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    /*
   * Gets the current yaw angle relative to the last reset.
   * @return
   */
    // BEGIN NAVX INIT AND CALIBRATION
    AHRS gyro = new AHRS(SPI.Port.kMXP);
       
  public void resetGyro(){   
      gyro.reset();
  }


  public double getRoll() {
    double rollAngleDegrees = gyro.getRoll();
    return rollAngleDegrees;
  }

    /**
   * Gets the yaw rate, or how quickly the robot is turning.
   * @return
   */
  public double getPitch() {
       double pitchAngleDegrees = gyro.getPitch();
       return pitchAngleDegrees;
  }

}
