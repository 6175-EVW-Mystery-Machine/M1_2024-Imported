package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final CANSparkFlex m_shooterLeft;
  private final CANSparkFlex m_shooterRight;
  private Blinkin m_blinkin;

  public Shooter() {
    m_shooterLeft = new CANSparkFlex(Constants.MyConstants.kShooterLeftCAN, MotorType.kBrushless);
    m_shooterRight = new CANSparkFlex(Constants.MyConstants.kShooterRightCAN, MotorType.kBrushless);

    m_shooterLeft.restoreFactoryDefaults();  
    m_shooterLeft.setInverted(false);
    m_shooterLeft.setIdleMode(IdleMode.kCoast);
    m_shooterLeft.burnFlash();

    m_shooterRight.restoreFactoryDefaults();  
    m_shooterRight.setInverted(false);
    m_shooterRight.setIdleMode(IdleMode.kCoast);
    m_shooterRight.burnFlash();

  }

  @Override
  public void periodic() {

  }

  //MY METHODS - IDK WHAT ITS ACTUALLY CALLED
  public Command c_startFlywheelAuto(double rSpeed, double lSpeed) {
    
    return new InstantCommand(() -> {
    m_shooterLeft.set(lSpeed);
    m_shooterRight.set(rSpeed);
    }, this);
  }

  public void c_startFlywheel(double rSpeed, double lSpeed) {
    
    m_shooterLeft.set(lSpeed);
    m_shooterRight.set(rSpeed);

  }

  public void c_stopFlywheel() {
    m_shooterLeft.set(0);
    m_shooterRight.set(0);
  }

  public void c_startFlywheelAxis(double axis, double uSpeed, double dSpeed) {
    if (axis >= 0.5) {
    m_shooterLeft.set(uSpeed);
    m_shooterRight.set(dSpeed);
    } else {
    m_shooterLeft.set(0);
    m_shooterRight.set(0);
    }
  
  }
}
