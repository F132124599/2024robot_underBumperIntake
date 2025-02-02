// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem m_swerveSubsystem;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier ySpeedFunc;
  private final DoubleSupplier zSpeedFunc;
  private final BooleanSupplier isSlowFunc;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;

  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private boolean isSlow;
  public ManualDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;
    this.isSlowFunc = isSlow;

    this.xLimiter = new SlewRateLimiter(4);
    this.yLimiter = new SlewRateLimiter(4);
    this.zLimiter = new SlewRateLimiter(4);


    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xSpeedFunc.getAsDouble();
    ySpeed = ySpeedFunc.getAsDouble();
    zSpeed = zSpeedFunc.getAsDouble();

    xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
    ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);
    zSpeed = MathUtil.applyDeadband(this.zSpeed, OperatorConstants.kJoystickDeadBand);

    xSpeed = xLimiter.calculate(this.xSpeed);
    ySpeed = yLimiter.calculate(this.ySpeed);
    zSpeed = zLimiter.calculate(this.zSpeed);

    isSlow = isSlowFunc.getAsBoolean();

    if(isSlow) {
      xSpeed = xSpeed*0.6;
      ySpeed = ySpeed*0.6;
      zSpeed = zSpeed*0.6;
    }else {
      xSpeed = xSpeed*0.8;
      ySpeed = ySpeed*0.8;
      zSpeed = zSpeed*0.8;
    }

    m_swerveSubsystem.drive(this.xSpeed, this.ySpeed, this.zSpeed,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
