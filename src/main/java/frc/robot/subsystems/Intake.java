package frc.robot.subsystems;
import frc.robot.Constants;

import javax.swing.plaf.TreeUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  
  private final CANSparkMax m_intakeDriver;
  private final DoubleSolenoid m_intakeExtension;
  
  public Intake() {
    m_intakeDriver = new CANSparkMax(Constants.MyConstants.kIntakeCAN, MotorType.kBrushless);
    m_intakeExtension = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  }

  @Override
  public void periodic() {
  
  }

  public Command c_intakeRun(double speed) {
    m_intakeDriver.set(speed);
    return null;
  }


  public void c_intakeControl(double speed, Boolean isExtended) {
   

    if (isExtended == true) {
      m_intakeExtension.set(DoubleSolenoid.Value.kForward);
    }
    if (isExtended == false) {
      m_intakeExtension.set(DoubleSolenoid.Value.kReverse);
    }
    else {
      m_intakeExtension.set(DoubleSolenoid.Value.kOff);
    }
    
    m_intakeDriver.set(speed);
  }

  public void c_startIntakeAxis(double axis, double speed) {
    if (axis >= 0.5) {
    m_intakeDriver.set(speed);
    } else {
    m_intakeDriver.set(0);
    }
  }
}
