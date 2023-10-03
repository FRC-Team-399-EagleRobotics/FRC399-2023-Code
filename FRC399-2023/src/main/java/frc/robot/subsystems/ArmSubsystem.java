package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.robot.constants.Constants.Arm;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor1, armMotor2;
    private final CANSparkMax wristMotor;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double wrist_kP, wrist_kI, wrist_kD, wrist_kIz, wrist_kFF, wrist_kMaxOutput, wrist_kMinOutput;
    private SparkMaxPIDController m_pidController;
    double encoderTicksPerDegree;

    public ArmSubsystem() {
        armMotor1 = new TalonFX(Arm.armMotor1_ID);
        armMotor2 = new TalonFX(Arm.armMotor2_ID);
        wristMotor = new CANSparkMax(Arm.wristMotor, MotorType.kBrushless);

        this.encoderTicksPerDegree = 227.56;

        int smoothing = 1;
        
        // Configure the Talon FX for position control
        armMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        armMotor1.setSensorPhase(false);
        armMotor1.setInverted(false);
        armMotor1.config_kF(0, 0);
        armMotor1.config_kP(0, 2.5);
        armMotor1.config_kI(0, 0);
        armMotor1.config_kD(0, 0);
        armMotor1.config_IntegralZone(0, 0);
        armMotor1.configClosedLoopPeakOutput(0, 1);
        armMotor1.configAllowableClosedloopError(0, 0);
        armMotor1.configMotionCruiseVelocity(11000, 0);
        armMotor1.configMotionAcceleration(22000, 0);

        armMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        armMotor2.setSensorPhase(false);
        armMotor2.setInverted(false);
        armMotor2.config_kF(0, 0);
        armMotor2.config_kP(0, 2.5);
        armMotor2.config_kI(0, 0);
        armMotor2.config_kD(0, 0);
        armMotor2.config_IntegralZone(0, 0);
        armMotor2.configClosedLoopPeakOutput(0, 1);
        armMotor2.configAllowableClosedloopError(0, 0);
        armMotor2.configMotionCruiseVelocity(11000, 0);
        armMotor2.configMotionAcceleration(15000, 0);

        

        // Current limiting & break mode
        armMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,35, 40, 1.0));
        armMotor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,35, 40, 0.5));
        armMotor1.setNeutralMode(NeutralMode.Brake);

        armMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,35, 40, 1.0));
        armMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,35, 40, 0.5));
        armMotor2.setNeutralMode(NeutralMode.Brake);
    

        // Motion Magic configuration
        armMotor1.configNominalOutputForward(0, Arm.kTimeoutMs);
		armMotor1.configNominalOutputReverse(0, Arm.kTimeoutMs);
		armMotor1.configPeakOutputForward(0.75, Arm.kTimeoutMs);
		armMotor1.configPeakOutputReverse(-0.30, Arm.kTimeoutMs);
        armMotor1.configMotionSCurveStrength(smoothing);
        armMotor1.setSelectedSensorPosition(0, 0, Arm.kTimeoutMs);

    

        //armMotor1.configClosedloopRamp()
        armMotor2.follow(armMotor1);

        m_pidController = wristMotor.getPIDController();
        m_encoder = wristMotor.getEncoder();
        wristMotor.setIdleMode(IdleMode.kBrake);

// PID coefficients for the wrist 
        wrist_kP = 0.1;
        wrist_kI = 0;
        wrist_kD = 0; 
        wrist_kIz = 0;
        wrist_kFF = 0; 
        wrist_kMaxOutput = .50; 
        wrist_kMinOutput = -.50;

// 
        m_pidController.setP(-wrist_kP);
        m_pidController.setI(wrist_kI);
        m_pidController.setD(wrist_kD);
        m_pidController.setIZone(wrist_kIz);
        m_pidController.setFF(wrist_kFF);
        m_pidController.setOutputRange(wrist_kMinOutput, wrist_kMaxOutput);
        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(0);
        m_pidController.setSmartMotionMaxVelocity(2000, 0);
        m_pidController.setSmartMotionMinOutputVelocity(0, 0);
        m_pidController.setSmartMotionMaxAccel(1500, 0);
        m_pidController.setSmartMotionAllowedClosedLoopError(1, 0);
    }

  // set PID vals for wristMotor

    public void setPosition(double positionDegrees) {
        // Convert the position in degrees to encoder ticks
        int positionTicks = (int) (270 * encoderTicksPerDegree);
        m_pidController.setReference(positionTicks, CANSparkMax.ControlType.kPosition);
        

        // Set the position setpoint for the ArmMotor1 FX
        armMotor1.set(ControlMode.Position, positionTicks);
    }

    public double getAngle() {
       double angle = armMotor1.getSelectedSensorPosition();

        return angle;
    }

    public void setKf(double kF) {
        armMotor1.config_kF(0, kF);
        
    }

    public void motionMagicTest() {
        int positionTicks = (int) (60 * encoderTicksPerDegree);
        armMotor1.set(TalonFXControlMode.MotionMagic, positionTicks);
    }

    public void cubeHigh() {
        int positionTicks = (int) (260 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(28, CANSparkMax.ControlType.kPosition);
    }

    public void coneHigh() {
        int positionTicks = (int) (290 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(32, CANSparkMax.ControlType.kPosition);
    }


    public void cubeMid() {
        int positionTicks = (int) (85 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(10, CANSparkMax.ControlType.kPosition);
    }

    public void coneMid() {
        int positionTicks = (int) (250 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(356, CANSparkMax.ControlType.kPosition);
    }

    public void cubeLow() {
        int positionTicks = (int) (15 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(10, CANSparkMax.ControlType.kPosition);

    }

    public void coneLow() {
        int positionTicks = (int) (15 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(15, CANSparkMax.ControlType.kPosition);
    }

    public void cubeLowIntake() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(21, CANSparkMax.ControlType.kPosition);
    }

    public void coneLowIntake() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(20, CANSparkMax.ControlType.kPosition);
    }

    public void cubeCharlesIntake() {
        int positionTicks = (int) (60 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(5, CANSparkMax.ControlType.kPosition);
    }

    public void coneCharlesIntake() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(10, CANSparkMax.ControlType.kPosition);
    }

    public void stow() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.MotionMagic, positionTicks);
        m_pidController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    public void manual(double i) {
        armMotor1.set(ControlMode.PercentOutput, i);
    }

    public void rest() {
        encoderTicksPerDegree = 227.56;
    }
}
