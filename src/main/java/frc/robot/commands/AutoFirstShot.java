// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPitch;
import frc.robot.subsystems.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFirstShot extends SequentialCommandGroup {
  /** Creates a new AutoFirstShot. */
  public AutoFirstShot(Shooter m_shooter, ShooterPitch m_shooterPitch, Transfer m_transfer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_shooterPitch.c_autoSP(-0.18),
      m_shooter.c_startFlywheelAuto(0.5, 0.5),
      new WaitCommand(1),
      m_transfer.c_runTransferAuto(1),
      new WaitCommand(0.5)
    );
  }
}
