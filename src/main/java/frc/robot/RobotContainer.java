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
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RotatorJog;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Transfer m_transfer = new Transfer();
  private final RotatorJog m_rotJog = new RotatorJog();
  private final Intake m_intake = new Intake();

  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    private final AutoBuilder autoBuilder = new AutoBuilder();

private final AutoScoreCommand autoScoreCommand =
    new AutoScoreCommand(scoreLocation, shooterSubsystem::hasCone, autoBuilder, m_robotDrive);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
      new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController.getLeftY())*0.75, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getLeftX())*0.75, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX())*0.5, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  
    m_transfer.setDefaultCommand(
      new RunCommand(
        () -> m_transfer.c_runTransfer(0), m_transfer));

    m_rotJog.setDefaultCommand(
      new RunCommand(
        () -> m_rotJog.c_rotJog(0), m_rotJog));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
    // new JoystickButton(m_driverController, OIConstants.kTriangleButton);
    // new JoystickButton(m_driverController, OIConstants.kSquareButton);
    // new JoystickButton(m_driverController, OIConstants.kCircleButtion);
    // new JoystickButton(m_driverController, OIConstants.kXButton);

    // new JoystickButton(m_driverController, OIConstants.kRightBumper);
    // new JoystickButton(m_driverController, OIConstants.kLeftBumper);
    // new JoystickButton(m_driverController, OIConstants.kRightTrigger);
    // new JoystickButton(m_driverController, OIConstants.kLeftTrigger);
    
    // new POVButton(m_driverController, OIConstants.kLeftArrowDeg);
    // new POVButton(m_driverController, OIConstants.kRightArrowDeg);
    // new POVButton(m_driverController, OIConstants.kUpArrowDeg);
    // new POVButton(m_driverController, OIConstants.kDownArrowDeg);


    //SET X BINDING
    // new JoystickButton(m_driverController, 3)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),  
    //         m_robotDrive));

    new JoystickButton(m_driverController, Button.kL2.value).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(1 , 1), m_shooter));
    new JoystickButton(m_driverController, Button.kL2.value).whileFalse(new RunCommand(() -> m_shooter.c_stopFlywheel(), m_shooter));

    new JoystickButton(m_driverController, Button.kR2.value).whileTrue(new RunCommand(() -> m_shooter.c_startFlywheel(-0.05 , -0.05), m_shooter));
    new JoystickButton(m_driverController, Button.kR2.value).whileFalse(new RunCommand(() -> m_shooter.c_stopFlywheel(), m_shooter));

    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(0.3), m_transfer));
    new JoystickButton(m_driverController, 3).whileFalse(new RunCommand(() -> m_transfer.c_runTransfer(0), m_transfer));

    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_transfer.c_runTransfer(-1), m_transfer));
    new JoystickButton(m_driverController, 1).whileFalse(new RunCommand(() -> m_transfer.c_runTransfer(0), m_transfer));


    //############ INPUT BUTTON BINDINGS #########################
    new JoystickButton(m_driverController, 4).whileTrue(new RunCommand(() -> m_rotJog.c_rotJog(-0.1), m_rotJog));
    new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() -> m_rotJog.c_rotJog(0.1), m_rotJog));

    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_intake.c_intakeRun(0.5), m_intake));
    new JoystickButton(m_driverController, 1).whileFalse(new RunCommand(() -> m_intake.c_intakeRun(0), m_intake));

    new JoystickButton(m_driverController, 10).onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }



 private void configureDashboard() {
    /**** Driver tab ****/
    var driverTab = Shuffleboard.getTab("Driver");
    
    Shuffleboard.getTab("Pre-round")
    .add("Auto Mode", autoModeChooser)
    .withSize(2, 1) // make the widget 2x1
    .withPosition(0, 0); // place it in the top-left corner

    driverTab.add(new HttpCamera("limelight-high", "http://10.70.28.13:1181"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", true, "showControls", false, "rotation", "QUARTER_CCW"))
        .withSize(4, 6).withPosition(0, 0);

    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");
  
    // Pose estimation
    PoseEstimator.withWidget(visionTab);

  }




  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}