// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPitch;
import frc.robot.subsystems.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoReadyShooter extends SequentialCommandGroup {
  // Intake m_intake = new Intake();
  // Transfer m_transfer = new Transfer();
  // Shooter m_shooter = new Shooter();
  // ShooterPitch m_shooterPitch = new ShooterPitch();
  public AutoReadyShooter(Intake m_intake, Transfer m_transfer, Shooter m_shooter, ShooterPitch m_shooterPitch) {
   
    addCommands(
      m_transfer.c_runTransferAuto(0),
      m_shooter.c_startFlywheelAuto(0.75, 0.75).withTimeout(0.5),
      m_transfer.c_runTransferAuto(-0.5).withTimeout(0.2),
      m_transfer.c_runTransferAuto(1)

    );
  }
}
