// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends Command {
  DriveSubsystem m_drive = new DriveSubsystem();
  public DriveForward() {
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(
                -MathUtil.applyDeadband((0.2), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
                true, true);
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
