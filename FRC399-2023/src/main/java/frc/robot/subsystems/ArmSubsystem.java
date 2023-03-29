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
    private final Solenoid extensionSolenoid,
//  wristSolenoid,
    intakeSolenoid;
    double encoderTicksPerDegree;

    public ArmSubsystem() {
        armMotor1 = new TalonFX(Arm.armMotor1_ID);
        armMotor2 = new TalonFX(Arm.armMotor2_ID);
        wristMotor = new CANSparkMax(Arm.wristMotor, MotorType.kBrushless);

        extensionSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.extensionSolenoid_ID);
//      wristSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.wristSolenoid_ID);
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Arm.clawSolenoid_ID);

        this.encoderTicksPerDegree = 227.56;

        int smoothing = 4;
        
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

        // Current limiting & break mode
        armMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,20, 25, 1.0));
        armMotor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10, 15, 0.5));
        armMotor1.setNeutralMode(NeutralMode.Brake);
    

        // Motion Magic configuration
        armMotor1.configNominalOutputForward(0, Arm.kTimeoutMs);
		armMotor1.configNominalOutputReverse(0, Arm.kTimeoutMs);
		armMotor1.configPeakOutputForward(1, Arm.kTimeoutMs);
		armMotor1.configPeakOutputReverse(-1, Arm.kTimeoutMs);
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
        wrist_kMaxOutput = .25; 
        wrist_kMinOutput = -.25;

// 
        m_pidController.setP(-wrist_kP);
        m_pidController.setI(wrist_kI);
        m_pidController.setD(wrist_kD);
        m_pidController.setIZone(wrist_kIz);
        m_pidController.setFF(wrist_kFF);
        m_pidController.setOutputRange(wrist_kMinOutput, wrist_kMaxOutput);
        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(0);
    }

  // set PID vals for wristMotor

    public void setPosition(double positionDegrees) {
        // Convert the position in degrees to encoder ticks
        int positionTicks = (int) (270 * encoderTicksPerDegree);
        m_pidController.setReference(positionTicks, CANSparkMax.ControlType.kPosition);
        

        // Set the position setpoint for the ArmMotor1 FX
        armMotor1.set(ControlMode.Position, positionTicks);
    }

    public void motionMagicTest() {
        int positionTicks = (int) (270 * encoderTicksPerDegree);
        armMotor1.set(TalonFXControlMode.MotionMagic, positionTicks);

    }

    public void intakeTest() {
        m_pidController.setReference(15, CANSparkMax.ControlType.kPosition);
    }

    public void intakeTest2() {
        m_pidController.setReference(-15, CANSparkMax.ControlType.kPosition);
    }

    public void high() {
        int positionTicks = (int) (270 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

//        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(true);
    }


    public void mid() {
        int positionTicks = (int) (225 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);
//        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(false);
    }

    public void cubeShooter() {
        int positionTicks = (int) (45 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

//        wristSolenoid.set(false);
        intakeSolenoid.set(false);
        extensionSolenoid.set(false);
    }

    public void highIntake() {
        int positionTicks = (int) (120 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

//        wristSolenoid.set(true);
        intakeSolenoid.set(true);
        extensionSolenoid.set(true);
    }

    public void lowIntake() {
        int positionTicks = (int) (30 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);

//       wristSolenoid.set(true);
        intakeSolenoid.set(false);
        extensionSolenoid.set(false);
    }

    public void stow() {
        int positionTicks = (int) (0 * encoderTicksPerDegree);
        armMotor1.set(ControlMode.Position, positionTicks);
        
//        wristSolenoid.set(false);
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
