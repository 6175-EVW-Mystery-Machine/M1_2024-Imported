package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotator extends SubsystemBase {

  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


public Rotator() {

    // initialize motor
    m_motor = new CANSparkMax(Constants.MyConstants.kRotatorCAN, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.1; 
    kMinOutput = -0.1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

}

@Override
public void periodic() {
// TO BE TUNED
double p = SmartDashboard.getNumber("P Gain", 0.1);
double i = SmartDashboard.getNumber("I Gain", 1e-4);
double d = SmartDashboard.getNumber("D Gain", 1);
double iz = SmartDashboard.getNumber("I Zone", 0);
double ff = SmartDashboard.getNumber("Feed Forward", 0);
double max = SmartDashboard.getNumber("Max Output", 0.1);
double min = SmartDashboard.getNumber("Min Output", -0.1);
double rotations = SmartDashboard.getNumber("Set Rotations", 0);

// if PID coefficients on SmartDashboard have changed, write new values to controller
if((p != kP)) { m_pidController.setP(p); kP = p; }
if((i != kI)) { m_pidController.setI(i); kI = i; }
if((d != kD)) { m_pidController.setD(d); kD = d; }
if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
if((max != kMaxOutput) || (min != kMinOutput)) { 
  m_pidController.setOutputRange(min, max); 
  kMinOutput = min; kMaxOutput = max; 
}

/**
 * PIDController objects are commanded to a set point using the 
 * SetReference() method.
 * 
 * The first parameter is the value of the set point, whose units vary
 * depending on the control type set in the second parameter.
 * 
 * The second parameter is the control type can be set to one of four 
 * parameters:
 *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
 *  com.revrobotics.CANSparkMax.ControlType.kPosition
 *  com.revrobotics.CANSparkMax.ControlType.kVelocity
 *  com.revrobotics.CANSparkMax.ControlType.kVoltage
 */
m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

}

public void c_rotatorToSetpoint(double setpoint) {
    m_pidController.setReference(setpoint, ControlType.kPosition);
}

public void c_rotJog(double speed) {
    m_motor.set(speed);
}
}