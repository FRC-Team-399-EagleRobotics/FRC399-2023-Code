/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * This is a demo program showing the use of the navX MXP to implement an
 * automatic balance feature, which can be used to help avoid a robot tipping
 * over when driving..
 *
 * The basic principle shown in the example is measurement of the Pitch
 * (rotation about the X axis) and Roll (rotation about the Y axis) angles. When
 * these angles exceed the "off balance" threshold and until these angles fall
 * below the "on balance" threshold, the drive system is automatically driven in
 * the opposite direction at a magnitude proportional to the Pitch or Roll
 * angle.
 *
 * Note that this is just a starting point for automatic balancing, and will
 * likely require a reasonable amount of tuning in order to work well with your
 * robot.
 */

public class Robot extends TimedRobot {
    // Two ways to call navX
    Gyro gyro;
    AHRS ahrs;

    DifferentialDrive _drive;
    Joystick stick;
    boolean autoBalanceXMode;
    boolean autoBalanceYMode;

    static final double kOffBalanceAngleThresholdDegrees = 15;
    static final double kOonBalanceAngleThresholdDegrees = 15;
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
     /* Talon motors */
     WPI_TalonSRX _backLeftMotor = new WPI_TalonSRX(1);
     WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(2);
     WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(3);
     WPI_TalonSRX _backRightMotor = new WPI_TalonSRX(4);
     
     // Bundling the motors
     MotorControllerGroup leftMotors = new MotorControllerGroup(_frontLeftMotor, _backLeftMotor);
     MotorControllerGroup rightMotors = new MotorControllerGroup(_frontRightMotor, _backRightMotor);

     /* Construct drivetrain by providing master motor controllers */
     _drive = new DifferentialDrive(leftMotors, rightMotors);

     stick = new Joystick(1);
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

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double xAxisRate = stick.getX();
        double yAxisRate = stick.getY();


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

         /*
                 //FRC Gryo method
        double rateAngleDegrees = gyro.getRate();
        double angleAngleDegrees = gyro.getAngle();
        if (!autoBalanceXMode && (Math.abs(rateAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        } else if (autoBalanceXMode && (Math.abs(rateAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if (!autoBalanceYMode && (Math.abs(rateAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        } else if (autoBalanceYMode && (Math.abs(rateAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }

        // Control drive system automatically,
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle

        if (autoBalanceXMode) {
            double pitchAngleRadians = rateAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        if (autoBalanceYMode) {
            double rollAngleRadians = angleAngleDegrees * (Math.PI / 180.0);
            yAxisRate = Math.sin(rollAngleRadians) * -1;
        }
*/
        //Run drivetrain
         try {
            /* 
             myRobot.driveCartesian(xAxisRate, yAxisRate, stick.getTwist(), 0); OG
             _drive.driveCartesian(xAxisRate, yAxisRate, stick.getTwist());
             Arcade drive with a given forward and turn rate */
           _drive.arcadeDrive(yAxisRate, xAxisRate);
         } catch (RuntimeException ex) {
             String err_string = "Drive system error:  " + ex.getMessage();
             DriverStation.reportError(err_string, true);
         }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
