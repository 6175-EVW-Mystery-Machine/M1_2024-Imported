// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OIConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private RobotContainer m_robotContainer;


  // private static final int deviceID = 20;
  // private CANSparkFlex m_motor;
  // private SparkPIDController m_pidController;
  // private RelativeEncoder m_encoder;
  // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();
  
   // initialize motor
  //  m_motor = new CANSparkFlex(deviceID, MotorType.kBrushless);

   /**
    * The RestoreFactoryDefaults method can be used to reset the configuration parameters
    * in the SPARK MAX to their factory default state. If no argument is passed, these
    * parameters will not persist between power cycles
    */
  //  m_motor.restoreFactoryDefaults();

   // initialze PID controller and encoder objects
  //  m_pidController = m_motor.getPIDController();
  //  m_encoder = m_motor.getEncoder();

   // PID coefficients
  //  kP = .0025; 
  //  kI = 0;
  //  kD = 0; 
  //  kIz = 0; 
  //  kFF = 0.000156; 
  //  kMaxOutput = .3 ; 
  //  kMinOutput = -.3;
  //  maxRPM = 5700;

   // Smart Motion Coefficients
  //  maxVel = 1500; // rpm
  //  maxAcc = 1000;

   // set PID coefficients
  //  m_pidController.setP(kP);
  //  m_pidController.setI(kI);
  //  m_pidController.setD(kD);
  //  m_pidController.setIZone(kIz);
  //  m_pidController.setFF(kFF);
  //  m_pidController.setOutputRange(kMinOutput, kMaxOutput);

   /**
    * Smart Motion coefficients are set on a SparkPIDController object
    * 
    * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
    * the pid controller in Smart Motion mode
    * - setSmartMotionMinOutputVelocity() will put a lower bound in
    * RPM of the pid controller in Smart Motion mode
    * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
    * of the pid controller in Smart Motion mode
    * - setSmartMotionAllowedClosedLoopError() will set the max allowed
    * error for the pid controller in Smart Motion mode
    */
  //  int smartMotionSlot = 0;
  //  m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  //  m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  //  m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  //  m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

   // display PID coefficients on SmartDashboard
  //  SmartDashboard.putNumber("P Gain", kP);
  //  SmartDashboard.putNumber("I Gain", kI);
  //  SmartDashboard.putNumber("D Gain", kD);
  //  SmartDashboard.putNumber("I Zone", kIz);
  //  SmartDashboard.putNumber("Feed Forward", kFF);
  //  SmartDashboard.putNumber("Max Output", kMaxOutput);
  //  SmartDashboard.putNumber("Min Output", kMinOutput);

  //  // display Smart Motion coefficients
  //  SmartDashboard.putNumber("Max Velocity", maxVel);
  //  SmartDashboard.putNumber("Min Velocity", minVel);
  //  SmartDashboard.putNumber("Max Acceleration", maxAcc);
  //  SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
  //  SmartDashboard.putNumber("Set Position", 0);
  //  SmartDashboard.putNumber("Set Velocity", 0);

  //  // button to toggle between velocity and smart motion modes
  //  SmartDashboard.putBoolean("Mode", true);
 }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    final double Ltrigger = m_driverController.getRawAxis(2);
    Constants.MyConstants.ktriggerL = Ltrigger;

    final double Rtrigger = m_driverController.getRawAxis(3);
    Constants.MyConstants.ktriggerR = Rtrigger;
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     // read PID coefficients from SmartDashboard
    //  double p = SmartDashboard.getNumber("P Gain", 0);
    //  double i = SmartDashboard.getNumber("I Gain", 0);
    //  double d = SmartDashboard.getNumber("D Gain", 0);
    //  double iz = SmartDashboard.getNumber("I Zone", 0);
    //  double ff = SmartDashboard.getNumber("Feed Forward", 0);
    //  double max = SmartDashboard.getNumber("Max Output", 0);
    //  double min = SmartDashboard.getNumber("Min Output", 0);
    //  double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    //  double minV = SmartDashboard.getNumber("Min Velocity", 0);
    //  double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    //  double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
 
    //  // if PID coefficients on SmartDashboard have changed, write new values to controller
    //  if((p != kP)) { m_pidController.setP(p); kP = p; }
    //  if((i != kI)) { m_pidController.setI(i); kI = i; }
    //  if((d != kD)) { m_pidController.setD(d); kD = d; }
    //  if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    //  if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    //  if((max != kMaxOutput) || (min != kMinOutput)) { 
    //    m_pidController.setOutputRange(min, max); 
    //    kMinOutput = min; kMaxOutput = max; 
    //  }
    //  if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    //  if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    //  if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    //  if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
 
    //  double setPoint, processVariable;
    //  boolean mode = SmartDashboard.getBoolean("Mode", false);
    //  if(mode) {
    //    setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    //    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    //    processVariable = m_encoder.getVelocity();
    //  } else {
    //    setPoint = SmartDashboard.getNumber("Set Position", 0);
    //    /**
    //     * As with other PID modes, Smart Motion is set by calling the
    //     * setReference method on an existing pid object and setting
    //     * the control type to kSmartMotion
    //     */
    //    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    //    processVariable = m_encoder.getPosition();
    //  }
     
    //  SmartDashboard.putNumber("SetPoint", setPoint);
    //  SmartDashboard.putNumber("Process Variable", processVariable);
    //  SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
   }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}