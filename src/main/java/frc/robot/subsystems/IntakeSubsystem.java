// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWheel;

  public IntakeSubsystem() {
    intakeWheel = new TalonFX(IntakeConstants.intakeWheel_ID);

    intakeWheel.setInverted(true);
    intakeWheel.setNeutralMode(NeutralModeValue.Coast);

  }

  public void noteIntake() {
    intakeWheel.setVoltage(IntakeConstants.intakewheelVoltage);
  }

  public void stopIntake() {
    intakeWheel.setVoltage(0);
  }


  public void noteOut() {
    intakeWheel.setVoltage(-IntakeConstants.intakewheelVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
