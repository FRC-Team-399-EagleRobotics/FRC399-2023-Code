package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.Constants.Arm;


public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor1, armMotor2;
    private final double encoderTicksPerDegree;

    // Variables to store state of arm
    double degree = 0;

    public ArmSubsystem() {
        armMotor1 = new TalonFX(Arm.armMotor1_ID);
        armMotor2 = new TalonFX(Arm.armMotor2_ID);

        this.encoderTicksPerDegree = 227.56;
        
        // Configure the Talon FX for position control
        armMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        armMotor1.setSensorPhase(false);
        armMotor1.setInverted(false);
        armMotor1.config_kF(0, 0);
        armMotor1.config_kP(0, 0.1);
        armMotor1.config_kI(0, 0);
        armMotor1.config_kD(0, 0);
        armMotor1.config_IntegralZone(0, 0);
        armMotor1.configClosedLoopPeakOutput(0, 1);
        armMotor1.configAllowableClosedloopError(0, 0);

        // Follow armMotor1
        armMotor2.follow(armMotor1);
    }

    public void setPosition(double positionDegrees) {
        // Convert the position in degrees to encoder ticks
        int positionTicks = (int) (positionDegrees * encoderTicksPerDegree);

        // Set the position setpoint for the ArmMotor1 FX
        armMotor1.set(ControlMode.Position, positionTicks);
        armMotor2.set(ControlMode.Position, positionTicks);
    }

    public void rest() {
        armMotor1.set(ControlMode.PercentOutput, 0);
    }

}
