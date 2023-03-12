package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.Constants.Arm;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor1, armMotor2;
    private final Solenoid extensionSolenoid, wristSolenoid, intakeSolenoid;
    double encoderTicksPerDegree;

    public ArmSubsystem() {
        armMotor1 = new TalonFX(Arm.armMotor1_ID);
        armMotor2 = new TalonFX(Arm.armMotor2_ID);

        extensionSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.extensionSolenoid_ID);
        wristSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.wristSolenoid_ID);
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.clawSolenoid_ID);

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
        //armMotor1.configClosedloopRamp()
        armMotor2.follow(armMotor1);

    }

  
    public void setPosition(double positionDegrees) {
        // Convert the position in degrees to encoder ticks
        int positionTicks = (int) (270 * encoderTicksPerDegree);

        // Set the position setpoint for the ArmMotor1 FX
        armMotor1.set(ControlMode.Position, positionTicks);
    }

    public void high() {
        int positionTicks = (int) (270 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(true);
    }


    public void mid() {
        int positionTicks = (int) (225 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(false);
    }

    public void cubeShooter() {
        int positionTicks = (int) (45 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

        wristSolenoid.set(false);
        intakeSolenoid.set(false);
        extensionSolenoid.set(false);
    }

    public void highIntake() {
        int positionTicks = (int) (120 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(true);
    }

    public void lowIntake() {
        int positionTicks = (int) (30 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

        wristSolenoid.set(true);
        intakeSolenoid.set(false);
        extensionSolenoid.set(false);
    }

    public void stow() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);
        
        wristSolenoid.set(false);
        intakeSolenoid.set(false);
        extensionSolenoid.set(false);
    }

    public void manual(double i) {
        armMotor1.set(ControlMode.PercentOutput, i);
    }

    public void rest() {
        encoderTicksPerDegree = 227.56;
    }
}
