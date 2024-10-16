// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Amp;
import frc.robot.commands.AmpDown;
import frc.robot.commands.AutoFirstShot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoReadyShooter;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HeightLimitingCommand;
import frc.robot.commands.HoardAuto;
import frc.robot.commands.RevSub;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetXDrive;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.StartFlywheels;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitchTest;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.RotatorAbsolute;
// import frc.robot.subsystems.RotatorJog;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPitch;
// import frc.robot.subsystems.Sweeper;
import frc.robot.subsystems.Transfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final Shooter m_shooter = new Shooter();
  private final Transfer m_transfer = new Transfer();

//   private final RotatorJog m_rotJog = new RotatorJog();
  private final Intake m_intake = new Intake();
//   private final Sweeper m_sweeper = new Sweeper();
  private final Rotator m_rotator = new Rotator();
  private final Blinkin m_blinkin = new Blinkin();
  private final Elevator m_elevator = new Elevator();
private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
private final RotatorAbsolute m_RotatorAbsolute = new RotatorAbsolute();
private final LimitSwitchTest m_LimitSwitchTest = new LimitSwitchTest();
private final ShooterPitch m_ShooterPitch = new ShooterPitch();

  private final SendableChooser<Command> autoChooser;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(1);
  XboxController m_testingController = new XboxController(2);

  public RobotContainer() {
    
    m_compressor.enableDigital();

    NamedCommands.registerCommand("AutoStartIntake", m_intake.c_intakeRunAuto(-0.75));
    NamedCommands.registerCommand("AutoStopIntake", m_intake.c_intakeRunAuto(0));
    NamedCommands.registerCommand("AutoStartFlywheels",m_shooter.c_startFlywheelAuto(0.75,0.75));
    NamedCommands.registerCommand("AutoStopFlywheels",m_shooter.c_startFlywheelAuto(0,0));
    NamedCommands.registerCommand("AutoStartTransfer", m_transfer.c_runTransferAuto(1.0));
    NamedCommands.registerCommand("AutoStopTransfer", m_transfer.c_runTransferAuto(0));
    NamedCommands.registerCommand("AutoRevTransfer", m_transfer.c_runTransferAuto(-0.1));
    NamedCommands.registerCommand("AutoTranTest", m_transfer.c_runTransferAuto(0.5));
    NamedCommands.registerCommand("AutoShooterTest",m_shooter.c_startFlywheelAuto(-0.1,-0.1));
    NamedCommands.registerCommand("AutoIntakeTest", m_intake.c_intakeRunAuto(-0.75));
    NamedCommands.registerCommand("AutoRotDown", m_LimitSwitchTest.c_runRotAuto(0.05).withTimeout(0.5));
    NamedCommands.registerCommand("AutoRotStop", m_LimitSwitchTest.c_runRotAuto(0));
    NamedCommands.registerCommand("AutoAimSP", m_LimitSwitchTest.c_runRotAuto(0));

    NamedCommands.registerCommand("PitchPickup", m_ShooterPitch.c_autoSP(-0.17).withTimeout(1));
    NamedCommands.registerCommand("PitchClose", m_ShooterPitch.c_autoSP(-0.11).withTimeout(1.5));
    NamedCommands.registerCommand("PitchCloseSide", m_ShooterPitch.c_autoSP(-0.095).withTimeout(1.5));

    NamedCommands.registerCommand("autoIntakeCombo", new AutoIntake(m_intake, m_transfer, m_shooter, m_ShooterPitch));
    NamedCommands.registerCommand("autoReadyShooter", new AutoReadyShooter(m_intake, m_transfer, m_shooter, m_ShooterPitch));
    NamedCommands.registerCommand("HoardAuto", new HoardAuto(m_intake, m_transfer, m_shooter, m_ShooterPitch));
    NamedCommands.registerCommand("autoShoot", new AutoShoot(m_transfer, m_ShooterPitch));
    NamedCommands.registerCommand("autoFirstShot", new AutoFirstShot(m_shooter, m_ShooterPitch, m_transfer));

    m_robotDrive = new DriveSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser 6175", autoChooser);

    configureButtonBindings();

    m_robotDrive.setDefaultCommand(

        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_transfer.setDefaultCommand(
            new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.c_startIntakeAxis(Constants.MyConstants.ktriggerR, 0.5), m_intake));

    // m_ShooterPitch.setDefaultCommand(new RunCommand(() -> m_ShooterPitch.RotToSP(0), m_ShooterPitch));
    
    // m_rotator.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_rotator.c_rotJog(0), m_rotator));

    // m_sweeper.setDefaultCommand(
    //     new RunCommand(() -> m_sweeper.c_controlSweeperAxis(Constants.MyConstants.ktriggerR, -0.5, true), m_sweeper));

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));
    // LOWER 24 - 16
    // UPPer 24 - 17

    m_blinkin.setDefaultCommand(new RunCommand(() -> m_blinkin.c_shooterBlinkin(), m_blinkin));

  }

  private void configureButtonBindings() {

    //SETPOINTS
    //LEFT BUMPER - PODIUM SIDE SHOT
    new JoystickButton(m_operatorController,1).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.1), m_ShooterPitch));
    //RIGHT BUMPER - CENTER STAGE(UNDER CHAIN)
    new JoystickButton(m_operatorController,2).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.085), m_ShooterPitch));
    //LEFT ARROW - "WING" NOT TESTED
    new POVButton(m_operatorController,270).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.06), m_ShooterPitch));
    //"B" BUTTON - RESET
    new JoystickButton(m_operatorController, 4).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(0.25), m_ShooterPitch));
    //"Y" BUTTON - TRAP SETPOINT
    // new Joystick(m_operatorController, 4).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(0.2), m_ShooterPitch));
    //RIGHT ARROW - OVER STAGE (BETWEEN OP WING AND CENTER) - SETPOINT AND FLYWHEEL
    new POVButton(m_operatorController,8).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(0.45, 0.45)));
    new POVButton(m_operatorController,8).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));

    new POVButton(m_driverController,90).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.45, 0.45),m_shooter));
    new POVButton(m_driverController,90).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));
    //"START" BUTTON - AMP CORNER SHOT (BUTTON NOT PUT IN CORRECTLY)
    new JoystickButton(m_operatorController, 3).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.09), m_ShooterPitch));
    //"MENU" BUTTON - BACKWARDS SUBWOOFER SHOT (BUTTON NOT PUT IN CORRECTLY)
    new JoystickButton(m_operatorController, 5).onTrue(new HeightLimitingCommand(m_ShooterPitch, m_elevator));
    new JoystickButton(m_operatorController, 6).onTrue(new AmpDown(m_ShooterPitch, m_elevator));
    // new JoystickButton(m_operatorController, 3).onTrue(m_ShooterPitch.c_autoSP(0.25));
    new JoystickButton(m_operatorController, 7).onTrue(new RevSub(m_ShooterPitch));
    


    // new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.02), m_ShooterPitch));
    new JoystickButton(m_driverController, 6).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.17), m_ShooterPitch));
    new JoystickButton(m_driverController, 1).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.17), m_ShooterPitch));
    // new JoystickButton(m_driverController, 1).onFalse(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.02), m_ShooterPitch));

    new JoystickButton(m_driverController, 6).onTrue(new RunCommand(() -> m_blinkin.c_intakeBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 6).onFalse(new RunCommand(() -> m_blinkin.c_shooterBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() -> m_blinkin.c_sourceBlinkin(), m_blinkin));
    // new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_blinkin.c_floorBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_blinkin.c_ampBlinkin(), m_blinkin));

    new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_ShooterPitch.Jog(0.75), m_ShooterPitch));
    new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_ShooterPitch.Jog(0), m_ShooterPitch));
    new POVButton(m_driverController, 180).whileTrue(new RunCommand(() -> m_ShooterPitch.Jog(-0.5), m_ShooterPitch));
    new POVButton(m_driverController, 180).whileFalse(new RunCommand(() -> m_ShooterPitch.Jog(0), m_ShooterPitch));

     new POVButton(m_operatorController, 0).whileTrue(new RunCommand(() -> m_ShooterPitch.Jog(0.75), m_ShooterPitch));
    new POVButton(m_operatorController, 0).whileFalse(new RunCommand(() -> m_ShooterPitch.Jog(0), m_ShooterPitch));
    new POVButton(m_operatorController, 180).whileTrue(new RunCommand(() -> m_ShooterPitch.Jog(-0.75), m_ShooterPitch));
    new POVButton(m_operatorController, 180).whileFalse(new RunCommand(() -> m_ShooterPitch.Jog(0), m_ShooterPitch));

    new JoystickButton(m_driverController, 4).whileTrue(new ShootSpeaker(m_robotDrive));
    new JoystickButton(m_driverController, 4).whileFalse(new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    // new POVButton(m_driverController, 90).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.73), m_blinkin));
    // new POVButton(m_driverController, 0).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.71), m_blinkin));
    // new POVButton(m_driverController, 180).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.75), m_blinkin));
    // new POVButton(m_driverController, 270).onTrue(new RunCommand((   )->
    // m_blinkin.c_lightSet(0.77), m_blinkin));

new JoystickButton(m_testingController, 5).onTrue(new HoardAuto(m_intake, m_transfer, m_shooter, m_ShooterPitch));
new JoystickButton(m_testingController, 2).onTrue(new AutoReadyShooter(m_intake, m_transfer, m_shooter, m_ShooterPitch));
new JoystickButton(m_testingController, 3).onTrue(new AutoShoot(m_transfer, m_ShooterPitch));
new JoystickButton(m_testingController, 4).onTrue(new AutoIntake(m_intake, m_transfer, m_shooter, m_ShooterPitch));
new JoystickButton(m_testingController, 1).onTrue(new AutoFirstShot(m_shooter, m_ShooterPitch, m_transfer));
// new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> m_rotator.c_rotJog(0.3), m_rotator));
// new POVButton(m_driverController, 0).onTrue(new RunCommand(() -> m_rotator.c_rotJog(-0.3), m_rotator));

// new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_setMotorSpeed(-0.2), m_LimitSwitchTest));

    // new JoystickButton(m_driverController, 1)
    //     .onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(0), m_rotator));whileTrue
    // new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-7), m_rotator));
    // new POVButton(m_driverController, 0).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-30), m_rotator));
    // new POVButton(m_driverController, 180).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-40), m_rotator));
    // new POVButton(m_driverController, 270)
    //     .onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(2), m_rotator));
    // new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(-0.3), m_LimitSwitchTest));
    //  new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));
    // new POVButton(m_driverController, 180).onFalse(new RunCommand(() -> m_ShooterPitch.RotToSP(0.70), m_ShooterPitch));

     new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(0.075, 0.4), m_shooter));
     new JoystickButton(m_driverController, 8).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheel(0, 0), m_shooter));
    //  new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(0.8), m_transfer));
     new JoystickButton(m_driverController, 8).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));

     //OPERATOR CONTROLS
    //  new POVButton(m_operatorController, 0).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(-0.4), m_LimitSwitchTest));
    //  new POVButton(m_operatorController, 0).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));
    // new POVButton(m_operatorController, 180).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.2);
    // }, m_LimitSwitchTest));
    //  new POVButton(m_operatorController, 180).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));

    // new JoystickButton(m_operatorController, 1).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.7);
    // }, m_LimitSwitchTest));
    // new JoystickButton(m_operatorController, 1).onFalse(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0);
    // }, m_LimitSwitchTest));
    //   new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_rotator.c_rotJog(0.3), m_rotator));
    //  new JoystickButton(m_driverController, 1).whileFalse(new RunCommand(() -> m_rotator.c_rotJog(0), m_rotator));
   
    new POVButton(m_driverController, 270).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(-0.06), m_ShooterPitch));
    // new POVButton(m_driverController, 270).onFalse(new RunCommand(() -> m_ShooterPitch.RotToSPGrav(), m_ShooterPitch));
   
    // new JoystickButton(m_driverController, 1).onTrue(new RunCommand(() -> m_ShooterPitch.RotToSP(0.3), m_ShooterPitch));
    // new JoystickButton(m_driverController, 1).onFalse(new RunCommand(() -> m_ShooterPitch.RotToSP(0), m_ShooterPitch));

    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_elevator.c_elevatorUp(), m_elevator));
    new JoystickButton(m_driverController, 3).whileFalse(new RunCommand(() -> m_elevator.c_elevatorDown(), m_elevator));

    // new JoystickButton(m_operatorController, 3).onTrue(new RunCommand(() -> m_elevator.c_elevatorUp(), m_elevator));
    // new JoystickButton(m_operatorController, 1).onFalse(new RunCommand(() -> m_elevator.c_elevatorDown(), m_elevator));
    // new JoystickButton(m_operatorController, 2).onFalse(new RunCommand(() -> m_ShooterPitch.Jog(-1), m_ShooterPitch).withTimeout(0.75));

    // new JoystickButton(m_driverController, 3)
    //     .whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(0.1, 0.6), m_shooter));
    new JoystickButton(m_driverController, 9).whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband((m_driverController.getLeftY()), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getLeftX()), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getRightX() * 0.8), OIConstants.kDriveDeadband),
            true, false),
        m_robotDrive));
    new JoystickButton(m_driverController, 9).whileFalse(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband((m_driverController.getLeftY() * 0.60), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getLeftX() * 0.60), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, OIConstants.kDriveDeadband),
            true, true),
        m_robotDrive));

    // new JoystickButton(m_driverController, OIConstants.kRightTrigger)
    //     .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(-0.2), m_transfer));
    // new JoystickButton(m_driverController, OIConstants.kRightTrigger)
    //     .whileFalse(new RunCommand(() -> m_transfer.c_runTransfer(0), m_transfer));
    // new JoystickButton(m_driverController, 4).whileTrue(new RunCommand(() -> m_intake.c_intakeRun(0.5), m_intake));
    // new JoystickButton(m_driverController, 4).whileFalse(new RunCommand(() -> m_intake.c_intakeRun(0), m_intake));

    new JoystickButton(m_driverController, OIConstants.kLeftBumper)
        .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(1.0), m_transfer));
    new JoystickButton(m_driverController, OIConstants.kLeftBumper)
        .whileFalse(new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));

    // new JoystickButton(m_driverController, OIConstants.kLeftBumper).whileTrue(new
    // RunCommand(() -> m_blinkin.c_lightSet(-0.31), m_blinkin));
    // new JoystickButton(m_driverController,
    // OIConstants.kLeftBumper).whileFalse(new RunCommand(() ->
    // m_blinkin.c_lightSet(0.77), m_blinkin));

    // new JoystickButton(m_driverController, 4)
    //     .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(-0.2), m_transfer));
    // new JoystickButton(m_driverController, 4).whileFalse(new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, 4).whileTrue(new RunCommand(() -> m_intake.c_intakeRun(0.5), m_intake));
    new JoystickButton(m_driverController, 4).whileFalse(new RunCommand(() -> m_intake.c_startIntakeAxis(Constants.MyConstants.ktriggerR, 0.5), m_intake));

    new JoystickButton(m_driverController, 6).whileTrue(new RunCommand(() -> m_intake.c_intakeRun(-0.75), m_intake));
    new JoystickButton(m_driverController, 6).whileFalse(new RunCommand(() -> m_intake.c_startIntakeAxis(Constants.MyConstants.ktriggerR, 0.5), m_intake));
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(0.75), m_transfer));
    new JoystickButton(m_driverController, 6).whileFalse(new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(-0.2, -0.2), m_shooter));
    
    // new JoystickButton(m_driverController, 6).onFalse(new RunCommand(() -> m_ShooterPitch.RotToSP(-0.05), m_ShooterPitch));

    new JoystickButton(m_driverController, 2)
        .whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(-0.2, -0.2), m_shooter));
    new JoystickButton(m_driverController, 2).whileFalse(
        new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));
    // new JoystickButton(m_driverController,
    // OIConstants.kTriangleButton).whileTrue(new RunCommand(() ->
    // m_rotator.c_rotJog(0.2), m_rotator));
    // new JoystickButton(m_driverController,
    // OIConstants.kTriangleButton).whileFalse(new RunCommand(() ->
    // m_rotator.c_rotJog(0), m_rotator));
    // new JoystickButton(m_driverController,
    // OIConstants.kSquareButton).whileTrue(new RunCommand(() ->
    // m_rotator.c_rotJog(-0.2), m_rotator));
    // new JoystickButton(m_driverController,
    // OIConstants.kSquareButton).whileFalse(new RunCommand(() ->
    // m_rotator.c_rotJog(0), m_rotator));
    new JoystickButton(m_driverController, 7).onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // ####### NOT FINISHED - AMP TOGGLE BUTTON ######
    // boolean toggle = false;

    // if (m_driverController.getRawButtonPressed(3)) {
    // if (toggle) {
    // new ParallelCommandGroup(Amp);
    // }
    // }

  }

  public Command getAutonomousCommand() {

    // final Pose2d m_centerPose = new Pose2d(1.5, 5.55, new Rotation2d(0));
    // m_robotDrive.resetOdometry(m_centerPose);
    return autoChooser.getSelected();
  }

  // public Command getAutonomousCommand() {
  // return new PathPlannerAuto("Example Auto");
  // }

//   public Command getAutonomousCommand() {    
//   // Create config for trajectory
//   TrajectoryConfig config = new TrajectoryConfig(
//   AutoConstants.kMaxSpeedMetersPerSecond,
//   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//   // Add kinematics to ensure max speed is actually obeyed
//   .setKinematics(DriveConstants.kDriveKinematics);

//   // An example trajectory to follow. All units in meters.
//   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//   // Start at the origin facing the +X direction
//   new Pose2d(0, 0, new Rotation2d(0)),
//   // Pass through these two interior waypoints, making an 's' curve path
//   List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//   // End 3 meters straight ahead of where we started, facing forward
//   new Pose2d(3, 0, new Rotation2d(0)),
//   config);

//   var thetaController = new ProfiledPIDController(
//   AutoConstants.kPThetaController, 0, 0,
//   AutoConstants.kThetaControllerConstraints);
//   thetaController.enableContinuousInput(-Math.PI, Math.PI);

//   SwerveControllerCommand swerveControllerCommand = new
//   SwerveControllerCommand(
//   exampleTrajectory,
//   m_robotDrive::getPose, // Functional interface to feed supplier
//   DriveConstants.kDriveKinematics,

//   // Position controllers
//   new PIDController(AutoConstants.kPXController, 0, 0),
//   new PIDController(AutoConstants.kPYController, 0, 0),
//   thetaController,
//   m_robotDrive::setModuleStates,
//   m_robotDrive);

//   // Reset odometry to the starting pose of the trajectory.
//   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//   // Run path following command, then stop at the end.
//   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
//   false, false));

  
//   }
}
