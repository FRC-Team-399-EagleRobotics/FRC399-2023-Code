package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX armMotor1, armMotor2;
    private CANSparkMax wristMotor;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double wrist_kP, wrist_kI, wrist_kD, wrist_kIz, wrist_kFF, wrist_kMaxOutput, wrist_kMinOutput;
    private SparkMaxPIDController m_pidController;
    double encoderTicksPerDegree;

    public ArmSubsystem() {

        configShoulder();
        configWrist();
        
        this.encoderTicksPerDegree = 227.56;

       
    }

    private void configShoulder() {

        // initialize shoulder motors at IDs
        armMotor1 = new TalonFX(Arm.armMotor1_ID);
        armMotor2 = new TalonFX(Arm.armMotor2_ID);
        
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

        // Configure motor 2 to follow 1
        //armMotor1.configClosedloopRamp()
        armMotor2.follow(armMotor1);

    }

    private void configWrist() {    
        // Initialize wrist motor at ID, config for brushless mode
        wristMotor = new CANSparkMax(Arm.wristMotor, MotorType.kBrushless); 
        wristMotor.restoreFactoryDefaults();

        // Configure pid controller
        m_pidController = wristMotor.getPIDController();
        m_encoder = wristMotor.getEncoder();
        wristMotor.setIdleMode(IdleMode.kBrake);

        //PID coefficients for the wrist 
        wrist_kP = 0.1;
        wrist_kI = 0;
        wrist_kD = 0; 
        wrist_kIz = 0;
        wrist_kFF = 0; 
        wrist_kMaxOutput = .50; 
        wrist_kMinOutput = -.50;

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

    public static enum PositionState {
        CUBE_LOW,
        CUBE_MID,
        CUBE_HIGH,
        
        CONE_LOW,
        CONE_MID,
        CONE_HIGH,

        CUBE_HP,
        CUBE_FLOOR,
        CONE_HP,
        CONE_FLOOR,

        STOW
    }

    public void setState(ArmSubsystem.PositionState state) {
        double shoulder = 0, wrist = 0;
        switch(state) {
            // Stow states - robot fully folded
            case STOW:
                shoulder = 0;
                wrist = 0;
                break;

            // cube scoring states
            case CUBE_LOW:
                shoulder = 15;
                wrist = 10;
                break;
            case CUBE_MID:
                shoulder = 85;
                wrist = 10;
                break;
            case CUBE_HIGH:
                shoulder = 260;
                wrist = 28;
                break;

            // cone scoring states
            case CONE_LOW:
                shoulder = 15;
                wrist = 15;
                break;
            case CONE_MID:
                shoulder = 250;
                wrist = 356;
                break;
            case CONE_HIGH:
                shoulder = 290;
                wrist = 32;
                break;

            // game piece loading states
            case CONE_HP:
                shoulder = 0;
                wrist = 10;
                break;
            case CONE_FLOOR:
                shoulder = 0;
                wrist = 20;
                break;
            case CUBE_HP:
                shoulder = 60;
                wrist = 5;
                break;
            case CUBE_FLOOR:
                shoulder = 0;
                wrist = 21;
                break;

            // if fed an invalid state, stow the arm
            default:
                shoulder = 0;
                wrist = 0;
                break;
        }

        // write positions to motor controllers. 
        setPosition(shoulder, wrist);

    }

    private void setPosition(double shoulder, double wrist) {
        // set shoulder position, convert to encoder ticks. 
        armMotor1.set(ControlMode.MotionMagic, (int) (shoulder * encoderTicksPerDegree));

        // set wrist position
        m_pidController.setReference(wrist, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() {
       double angle = armMotor1.getSelectedSensorPosition();

        return angle;
    }

    public void printAngles() {
        System.out.println("S: " + armMotor1.getSelectedSensorPosition() + "\tW: " + m_encoder.getPosition());
    }

    public void manual(double i) {
        armMotor1.set(ControlMode.PercentOutput, i);
    }

    public void rest() {
        encoderTicksPerDegree = 227.56;
    }
}
