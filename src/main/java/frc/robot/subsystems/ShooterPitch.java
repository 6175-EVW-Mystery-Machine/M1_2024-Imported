// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPitch extends SubsystemBase {
  
  private final CANSparkFlex m_aimMotor = new CANSparkFlex(10, MotorType.kBrushless);
  private AbsoluteEncoder m_aimEncoder;

  private final double AIM_OFFSET = 0.3268059;

  private final int MEASUREDPOSHORIZONTAL = 0;
  private final double TICKSPERDEGREE = 4096 / 360;
  
  // <<<<< SET WITH REV CLIENT >>>>>
  private final double AIM_kP = 4.0;
  private final double AIM_kI = 0;
  private final double AIM_kD = 0.05;

  private final Measure<Voltage> maxGravityFF = Units.Volts.of(0.25);

  private SparkPIDController aimPIDController = m_aimMotor.getPIDController();

  private double AIM;
  private int currentPos;

  public ShooterPitch() {
     m_aimEncoder = m_aimMotor.getAbsoluteEncoder(Type.kDutyCycle);
     
      m_aimEncoder.setZeroOffset(AIM_OFFSET);
      m_aimEncoder.setInverted(false);

     aimPIDController.setP(AIM_kP, 0);
     aimPIDController.setI(AIM_kI, 0);
     aimPIDController.setD(AIM_kD, 0);



    //  aimPIDController.setOutputRange(-0.5, 0.5);
    aimPIDController.setPositionPIDWrappingMaxInput(1);
    aimPIDController.setPositionPIDWrappingMinInput(0);

     aimPIDController.setFeedbackDevice(m_aimEncoder);

     aimPIDController.setPositionPIDWrappingEnabled(true);

   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putData("ENCODER");
    // currentPos = aimPIDController.getPosition();
    //  double degrees = (currentPos - MEASUREDPOSHORIZONTAL) / TICKSPERDEGREE;


  }


  //SP Method Bug <<<<<<<< non public var? >>>>>>>>
  public void RotToSP(double setpoint) {
    aimPIDController.setReference(setpoint, ControlType.kPosition);
  }

  public void RotToSPGrav ( double setpoint ) {
   double cosineScalar = Math.cos(2*Math.PI*(m_aimEncoder.getPosition()));
   var gravityFF = maxGravityFF.in(Units.Volts) * cosineScalar;

    aimPIDController.setReference(setpoint, ControlType.kPosition, 0, gravityFF, ArbFFUnits.kVoltage);
    AIM = setpoint;
  }

   public Command c_autoSP(double setpoint) {
    return new InstantCommand(() -> {
        double cosineScalar = Math.cos(2*Math.PI*(m_aimEncoder.getPosition()));
   var gravityFF = maxGravityFF.in(Units.Volts) * cosineScalar;

    aimPIDController.setReference(setpoint, ControlType.kPosition, 0, gravityFF, ArbFFUnits.kVoltage);
    AIM = setpoint;
        }, this);
  }

  public void Jog (double SPEED) {
    m_aimMotor.set(SPEED);
  }

  public void AfterAmp () {
    m_aimMotor.set(-0.5);
  }

  public void isAtTarget() {
    // if (AIM == currentPos.getAsDouble)
  }
}
