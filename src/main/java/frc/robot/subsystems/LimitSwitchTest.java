// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitchTest extends SubsystemBase {
   private CANSparkFlex m_motor;
   DigitalInput toplimitSwitch = new DigitalInput(3);
  public LimitSwitchTest() {
m_motor = new CANSparkFlex(80, MotorType.kBrushless);

m_motor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("button", toplimitSwitch.get());
  }

  public void c_setMotorSpeed(double speed) {
    if (speed >= 0) {
        if (!toplimitSwitch.get()) {

            // We are going up and top limit is tripped so stop
            m_motor.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            m_motor.set(speed);
        }
      } 
  }

  public Command c_runRotAuto(double speed) {
    return new RunCommand(() -> {
        m_motor.set(speed);
        }, this);
  }

  public void c_rotJog(double speed) {
    m_motor.set(speed);
}
    /*
    else {
        if (bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
            m_motor.set(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
            m_motor.set(speed);
        }
    }
    */
}
