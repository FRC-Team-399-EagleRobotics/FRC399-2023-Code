package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.Constants.Arm;


public class ArmSubsystem extends SubsystemBase {
    private final TalonFX ArmMotor1, ArmMotor2;
    private final double encoderTicksPerDegree;

    public ArmSubsystem() {
        ArmMotor1 = new TalonFX(Arm.ArmMotor1_ID);
        ArmMotor2 = new TalonFX(Arm.ArmMotor2_ID);

        this.encoderTicksPerDegree = 227.56;
        
        // Configure the Talon FX for position control
        ArmMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        ArmMotor1.setSensorPhase(false);
        ArmMotor1.setInverted(false);
        ArmMotor1.config_kF(0, 0);
        ArmMotor1.config_kP(0, 0.1);
        ArmMotor1.config_kI(0, 0);
        ArmMotor1.config_kD(0, 0);
        ArmMotor1.config_IntegralZone(0, 0);
        ArmMotor1.configClosedLoopPeakOutput(0, 1);
        ArmMotor1.configAllowableClosedloopError(0, 0);

        // Configure the Talon FX for position control
        ArmMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        ArmMotor2.setSensorPhase(false);
        ArmMotor2.setInverted(false);
        ArmMotor2.config_kF(0, 0);
        ArmMotor2.config_kP(0, 0.1);
        ArmMotor2.config_kI(0, 0);
        ArmMotor2.config_kD(0, 0);
        ArmMotor2.config_IntegralZone(0, 0);
        ArmMotor2.configClosedLoopPeakOutput(0, 1);
        ArmMotor2.configAllowableClosedloopError(0, 0);
    }

    public void setPosition(double positionDegrees) {
        // Convert the position in degrees to encoder ticks
        int positionTicks = (int) (positionDegrees * encoderTicksPerDegree);

        // Set the position setpoint for the ArmMotor1 FX
        ArmMotor1.set(ControlMode.Position, positionTicks);
        ArmMotor2.set(ControlMode.Position, -positionTicks);
    }
}
