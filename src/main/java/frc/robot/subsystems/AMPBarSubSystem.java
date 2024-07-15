// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.core.io.OutputDecorator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimBarConstants;

public class AMPBarSubSystem extends SubsystemBase {
  /** Creates a new AimBarSubSystem. */
  private final TalonFX aimBarArm;

  private final PIDController armPid;

  private final ArmFeedforward armFeedforward;

  private double pidOutPut;
  private double feedForwardOutPut;
  private double OutPut;

  private double arriveAngle;
  public AMPBarSubSystem() {
    aimBarArm = new TalonFX(AimBarConstants.aimbarArmID);

    aimBarArm.setInverted(false);

    aimBarArm.setNeutralMode(NeutralModeValue.Brake);

    armPid = new PIDController(AimBarConstants.armPid_kp, AimBarConstants.armPid_ki, AimBarConstants.armPid_kd);

    armFeedforward = new ArmFeedforward(AimBarConstants.armFeedForward_ks, AimBarConstants.armFeedForward_kg, AimBarConstants.armFeedForward_kv);
  }


  public double getAngle() {
    return aimBarArm.getDutyCycle().getValueAsDouble()*90;
  }

  public double getRadians() {
    return aimBarArm.getDutyCycle().getValueAsDouble()*Math.PI/2f;//*90*PI/180
  }

  public double getVelocity() {
    return aimBarArm.getVelocity().getValueAsDouble()*60;
  }

  public void setOutAngle() {
    arriveAngle = AimBarConstants.outAngle;
  }

  public void setBackAngle() {
    arriveAngle = AimBarConstants.backAngle;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AimArmAngle", getAngle());
    SmartDashboard.putNumber("AImArmVelocity", getVelocity());

    pidOutPut = armPid.calculate(getAngle(), arriveAngle);
    pidOutPut = Constants.setMaxOutPut(pidOutPut, AimBarConstants.maxOutPut);

    feedForwardOutPut = armFeedforward.calculate(getRadians(), 0);
    feedForwardOutPut = Constants.setMaxOutPut(feedForwardOutPut, AimBarConstants.maxOutPut);

    OutPut = feedForwardOutPut+pidOutPut;
    
    aimBarArm.set(OutPut);
  }
}
