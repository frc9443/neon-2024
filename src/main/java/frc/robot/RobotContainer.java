// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ActivateCompressorCommand;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.MoveShooterCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.speedAdjustCommand;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.VisionUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem m_robotDrive;
  private ShooterSubsystem m_ShooterSubsystem;
  private CompressorSubsystem m_CompressorSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final AHRS m_gyro;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_gyro = new AHRS(SerialPort.Port.kUSB);
    m_robotDrive = new DriveSubsystem(m_gyro);
    m_ShooterSubsystem = new ShooterSubsystem();
    m_CompressorSubsystem = new CompressorSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
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
    //Turns on Compressor when under 70 PSI and off at 90 PSI
    new Trigger(m_CompressorSubsystem.morePressureNeeded())
    .onTrue(new ActivateCompressorCommand(m_CompressorSubsystem));


    // Turn to 180 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kA.value)
    .onTrue(new TurnToAngleCommand(() -> 180, m_robotDrive).withTimeout(3));
    
    new JoystickButton(m_driverController, Button.kX.value)
    .onTrue(new TurnToAngleCommand(() -> -135, m_robotDrive).withTimeout(3));
    
    new JoystickButton(m_driverController, Button.kB.value)
    .onTrue(new TurnToAngleCommand(() -> 135, m_robotDrive).withTimeout(3));

    // Turn to 0 degrees when the 'B' button is pressed, with a 3 second timeout
    new JoystickButton(m_driverController, Button.kY.value)
    .onTrue(new TurnToAngleCommand(() -> 0, m_robotDrive).withTimeout(3));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .onTrue(new speedAdjustCommand(m_robotDrive, true));
    
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .onTrue(new speedAdjustCommand(m_robotDrive, false));


  
    
    // Activates  Shooter for 3 seconds. hopefully.
     new JoystickButton(m_OperatorController, Button.kY.value)
    .onTrue(new ShootCommand(m_ShooterSubsystem).withTimeout(3));

    new POVButton(m_OperatorController, 0)
    .onTrue(new MoveShooterCommand(m_ShooterSubsystem, true));
    
    new POVButton(m_OperatorController, 180)
    .onTrue(new MoveShooterCommand(m_ShooterSubsystem, false));

    // new JoystickButton(m_OperatorController, Button.kRightBumper.value)
    // .onTrue(new ActiveIntakeCommand(m_IntakeSubsystem, ));

    // Manual Overrides for stick control of intake arm and climber
    // new JoystickButton(m_OperatorController, Button.kLeftBumper.value)
    // .onTrue(new ManualOverrideCommand(m_IntakeArmSubsystem, m_ClimberSubsystem));

    new JoystickButton(m_OperatorController, Button.kA.value)
    .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, .3).withTimeout(3));

    new JoystickButton(m_OperatorController, Button.kB.value)
    .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, -.5).withTimeout(3));

    new JoystickButton(m_OperatorController, Button.kX.value)
    .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, -.8).withTimeout(3));

    new POVButton(m_OperatorController, 90)
    .onTrue(new TurnToAngleCommand(() -> VisionUtils.calculateNoteAngle(m_gyro), m_robotDrive).withTimeout(3));
    
    new POVButton(m_OperatorController, 180)
    .onTrue(new TurnToAngleCommand(() -> VisionUtils.calculateNoteAngle(m_gyro), m_robotDrive).withTimeout(3));
    





  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new FollowAprilTagCommand(m_robotDrive);
  }

  public Command getOldAutonomousCommand() {
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