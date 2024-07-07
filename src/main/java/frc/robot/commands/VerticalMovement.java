// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class VerticalMovement extends Command {
  /** Creates a new climbUp. */
  private final ClimberSubsystem m_climberSubsystem;
  private DoubleSupplier leftClimbSpeed;
  private DoubleSupplier rightClimbSpeed;

  private BooleanSupplier climberInsurance;
  public VerticalMovement(ClimberSubsystem climberSubsystem, DoubleSupplier leftClimbSpeed, DoubleSupplier rightClimbSpeed, BooleanSupplier climberInsurance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climberSubsystem = climberSubsystem; 
    this.leftClimbSpeed = leftClimbSpeed;
    this.rightClimbSpeed = rightClimbSpeed;
    this.climberInsurance = climberInsurance;

    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climberInsurance.getAsBoolean()){
      m_climberSubsystem.leftClimb(leftClimbSpeed.getAsDouble());
      m_climberSubsystem.rightClimb(rightClimbSpeed.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
