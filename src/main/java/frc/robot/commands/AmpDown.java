// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPitch;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpDown extends SequentialCommandGroup {
  /** Creates a new AmpDown. */
  public AmpDown(ShooterPitch m_ShooterPitch, Elevator m_elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    m_elevator.com_elevatorDown(),
    m_ShooterPitch.c_autoSP(0.25),
    new WaitCommand(1),
    m_ShooterPitch.c_autoSP(-0.17)
      
    );
  }
}
