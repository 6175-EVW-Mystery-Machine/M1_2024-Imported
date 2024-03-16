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
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetXDrive;
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

  private final SendableChooser<Command> autoChooser;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(1);

  public RobotContainer() {

    
    m_compressor.enableDigital();


    NamedCommands.registerCommand("AutoStartIntake", m_intake.c_intakeRunAuto(-0.75));
    NamedCommands.registerCommand("AutoStopIntake", m_intake.c_intakeRunAuto(0));
    NamedCommands.registerCommand("AutoStartFlywheels",
    m_shooter.c_startFlywheelAuto(0.9,0.9));
    NamedCommands.registerCommand("AutoStopFlywheels",
    m_shooter.c_startFlywheelAuto(0,0));
    NamedCommands.registerCommand("AutoStartTransfer", m_transfer.c_runTransferAuto(0.3));
    NamedCommands.registerCommand("AutoStopTransfer", m_transfer.c_runTransferAuto(0));
    NamedCommands.registerCommand("AutoRevTransfer", m_transfer.c_runTransferAuto(-0.1));

    NamedCommands.registerCommand("AutoTranTest", m_transfer.c_runTransferAuto(0.5));
    NamedCommands.registerCommand("AutoShooterTest",
    m_shooter.c_startFlywheelAuto(-0.1,-0.1));
    NamedCommands.registerCommand("AutoIntakeTest", m_intake.c_intakeRunAuto(-0.75));
    NamedCommands.registerCommand("AutoRotDown", m_LimitSwitchTest.c_runRotAuto(0.05).withTimeout(0.5));
    NamedCommands.registerCommand("AutoRotStop", m_LimitSwitchTest.c_runRotAuto(0));

    
    m_robotDrive = new DriveSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser 6175", autoChooser);
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(

        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY()) * 0.60, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX()) * 0.60, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX()) * 0.8, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_transfer.setDefaultCommand(
            new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.c_startIntakeAxis(Constants.MyConstants.ktriggerR, 0.5), m_intake));
    
    // m_rotator.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_rotator.c_rotJog(0), m_rotator));

    // m_sweeper.setDefaultCommand(
    //     new RunCommand(() -> m_sweeper.c_controlSweeperAxis(Constants.MyConstants.ktriggerR, -0.5, true), m_sweeper));

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.9, 0.9), m_shooter));
    // LOWER 24 - 16
    // UPPer 24 - 17

    m_blinkin.setDefaultCommand(new RunCommand(() -> m_blinkin.c_shooterBlinkin(), m_blinkin));

  }

  private void configureButtonBindings() {

    new JoystickButton(m_driverController, 6).onTrue(new RunCommand(() -> m_blinkin.c_intakeBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 6).onFalse(new RunCommand(() -> m_blinkin.c_shooterBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() -> m_blinkin.c_sourceBlinkin(), m_blinkin));
    // new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_blinkin.c_floorBlinkin(), m_blinkin));
    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_blinkin.c_ampBlinkin(), m_blinkin));
    // new POVButton(m_driverController, 90).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.73), m_blinkin));
    // new POVButton(m_driverController, 0).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.71), m_blinkin));
    // new POVButton(m_driverController, 180).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.75), m_blinkin));
    // new POVButton(m_driverController, 270).onTrue(new RunCommand(()->
    // m_blinkin.c_lightSet(0.77), m_blinkin));

// new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> m_rotator.c_rotJog(0.3), m_rotator));
// new POVButton(m_driverController, 0).onTrue(new RunCommand(() -> m_rotator.c_rotJog(-0.3), m_rotator));

// new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_setMotorSpeed(-0.2), m_LimitSwitchTest));

    // new JoystickButton(m_driverController, 1)
    //     .onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(0), m_rotator));
    // new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-7), m_rotator));
    // new POVButton(m_driverController, 0).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-30), m_rotator));
    // new POVButton(m_driverController, 180).onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(-40), m_rotator));
    // new POVButton(m_driverController, 270)
    //     .onTrue(new RunCommand(() -> m_rotator.c_rotatorToSetpoint(2), m_rotator));
    new POVButton(m_driverController, 0).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(-0.3), m_LimitSwitchTest));
     new POVButton(m_driverController, 0).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));
    new POVButton(m_driverController, 180).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.2);  
    }, m_LimitSwitchTest));
     new POVButton(m_driverController, 180).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));

     new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(0.025, 0.4), m_shooter));
     new JoystickButton(m_driverController, 8).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheel(0, 0), m_shooter));
     new JoystickButton(m_driverController, 8).whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(0.8), m_transfer));
     new JoystickButton(m_driverController, 8).whileFalse(new RunCommand(() -> m_shooter.c_startFlywheelAxis(Constants.MyConstants.ktriggerL, 0.75, 0.75), m_shooter));

     //OPERATOR CONTROLS
     new POVButton(m_operatorController, 0).whileTrue(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(-0.4), m_LimitSwitchTest));
     new POVButton(m_operatorController, 0).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));
    new POVButton(m_operatorController, 180).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.2);
    }, m_LimitSwitchTest));
     new POVButton(m_operatorController, 180).whileFalse(new RunCommand(() -> m_LimitSwitchTest.c_rotJog(0), m_LimitSwitchTest));

    new JoystickButton(m_operatorController, 1).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.7);
    }, m_LimitSwitchTest));
    new JoystickButton(m_operatorController, 1).onFalse(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0);
    }, m_LimitSwitchTest));
    //   new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_rotator.c_rotJog(0.3), m_rotator));
    //  new JoystickButton(m_driverController, 1).whileFalse(new RunCommand(() -> m_rotator.c_rotJog(0), m_rotator));
    new JoystickButton(m_driverController, 1).onTrue(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0.7);
    }, m_LimitSwitchTest));
    new JoystickButton(m_driverController, 1).onFalse(new RunCommand(() -> {m_LimitSwitchTest.c_setMotorSpeed(0);
    }, m_LimitSwitchTest));


    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_elevator.c_elevatorUp(), m_elevator));
    new JoystickButton(m_driverController, 3).whileFalse(new RunCommand(() -> m_elevator.c_elevatorDown(), m_elevator));

    // new JoystickButton(m_driverController, 3)
    //     .whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(0.1, 0.6), m_shooter));
    new JoystickButton(m_driverController, 9).whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband((m_driverController.getLeftY()), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getLeftX()), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getRightX()), OIConstants.kDriveDeadband),
            true, true),
        m_robotDrive));
    new JoystickButton(m_driverController, 9).whileFalse(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband((m_driverController.getLeftY()) * 0.6, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband((m_driverController.getLeftX()) * 0.6, OIConstants.kDriveDeadband),
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
        .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(0.5), m_transfer));
    new JoystickButton(m_driverController, OIConstants.kLeftBumper)
        .whileFalse(new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));

    // new JoystickButton(m_driverController, OIConstants.kLeftBumper).whileTrue(new
    // RunCommand(() -> m_blinkin.c_lightSet(-0.31), m_blinkin));
    // new JoystickButton(m_driverController,
    // OIConstants.kLeftBumper).whileFalse(new RunCommand(() ->
    // m_blinkin.c_lightSet(0.77), m_blinkin));

    new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(-0.2), m_transfer));
    new JoystickButton(m_driverController, 4).whileFalse(new RunCommand(() -> m_transfer.c_startTransferAxis(Constants.MyConstants.ktriggerR, -0.2), m_transfer));
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
    return autoChooser.getSelected();
  }

  // public Command getAutonomousCommand() {
  // return new PathPlannerAuto("Example Auto");
  // }

  // public Command getAutonomousCommand() {
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false, false));
  // }
}
