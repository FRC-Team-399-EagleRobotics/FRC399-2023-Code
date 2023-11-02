// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax wristmotor;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    public double wrist_kP, wrist_kI, wrist_kD, wrist_kIz, wrist_kFF, wrist_kMaxOutput, wrist_kMinOutput;
    private SparkMaxPIDController m_pidController;
}