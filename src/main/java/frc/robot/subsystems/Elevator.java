// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final DoubleSolenoid m_elevatorDoubleSolenoid;

  public Elevator() {

    m_elevatorDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

  }

  @Override
  public void periodic() {}

  public void c_elevatorUp() {
    m_elevatorDoubleSolenoid.set(Value.kForward);
  }

   public void c_elevatorDown() {
    m_elevatorDoubleSolenoid.set(Value.kReverse);
  }
}
