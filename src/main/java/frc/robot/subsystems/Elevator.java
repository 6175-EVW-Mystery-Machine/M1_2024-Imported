// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final DoubleSolenoid m_elevatorDoubleSolenoidLeft;
  private final DoubleSolenoid m_elevatorDoubleSolenoidRight;

  public Elevator() {

    m_elevatorDoubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);
    m_elevatorDoubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);

  }

  @Override
  public void periodic() {}

  public void c_elevatorUp() {
    m_elevatorDoubleSolenoidLeft.set(Value.kForward);
    m_elevatorDoubleSolenoidRight.set(Value.kForward);
  }

   public void c_elevatorDown() {
    m_elevatorDoubleSolenoidLeft.set(Value.kReverse);
    m_elevatorDoubleSolenoidRight.set(Value.kReverse);
  }
}
