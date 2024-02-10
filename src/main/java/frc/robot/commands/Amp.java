package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rotator;

public class Amp extends ParallelCommandGroup { 
  public Amp(Elevator elevator, Rotator rotator) {
    addCommands(
    
    new ElevatorUp(elevator),
    new RotatorToAmp(rotator)

    );
  }
}
