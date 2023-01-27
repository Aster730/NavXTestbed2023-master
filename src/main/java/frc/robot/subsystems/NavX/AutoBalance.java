// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NavX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

public class AutoBalance extends CommandBase {
  private RevDrivetrain drive;
  private AHRS ahrs;
  
  boolean balanceMode;

  private PIDController balanceCorrector = 
  new PIDController(balanceCorrection.kP, balanceCorrection.kI, balanceCorrection.kD);

  /** Creates a new AutoBalance. */
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.

    balanceCorrector.setTolerance(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceCorrector.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = ahrs.getPitch();

    drive.getDifferentialDrive().arcadeDrive(balanceCorrector.calculate(pitchAngleDegrees, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
