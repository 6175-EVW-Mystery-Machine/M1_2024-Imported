// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final DoubleSolenoid m_elevatorDoubleSolenoid;

  public Elevator() {

    m_elevatorDoubleSolenoid = new DoubleSolenoid(7, PneumaticsModuleType.REVPH, 6, 7);


    m_elevatorDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

}
  

  @Override
  public void periodic() {}

  public void c_elevatorUp() {
    m_elevatorDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

   public void c_elevatorDown() {
    m_elevatorDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public Command com_elevatorUp() {
    
    return new InstantCommand(() -> {
       m_elevatorDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }, this);
  }

  public Command com_elevatorDown() {
    
    return new InstantCommand(() -> {
      m_elevatorDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }, this);
  }
}
