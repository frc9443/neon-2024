// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.ManualOverrideCommand;
import frc.robot.commands.DropShooterAngleCommand;
import frc.robot.commands.RestartGyroCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.speedAdjustCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.VisionUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private DriveSubsystem m_DriveSubsystem;
  private ShooterSubsystem m_ShooterSubsystem;
  private CompressorSubsystem m_CompressorSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  private IntakeArmSubsystem m_IntakeArmSubsystem;
  private ClimberSubsystem m_ClimberSubsystem;

  // private final Command m_TurnShootAuto;
  // A complex auto routine that drives forward, drops a hatch, and then drives
  // backward.
  private final Command m_ShootAuto;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final AHRS m_gyro;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // For USB gyro (Neon)
    // m_gyro = new AHRS(SerialPort.Port.kUSB);
    // For MXP gyro card (Helium)
    m_gyro = new AHRS(SPI.Port.kMXP);

    m_DriveSubsystem = new DriveSubsystem(m_gyro);
    m_ShooterSubsystem = new ShooterSubsystem();
    m_CompressorSubsystem = new CompressorSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_IntakeArmSubsystem = new IntakeArmSubsystem();
    m_ClimberSubsystem = new ClimberSubsystem();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_DriveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_DriveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_DriveSubsystem));

    m_ShootAuto = new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem);
    // m_TurnShootAuto = Commands.sequence(
    // new TurnToAngleCommand(()-> -135, m_robotDrive),
    // new DriveCommand(m_robotDrive, .2, 0, 0, false).withTimeout(1),
    // m_ShootAuto
    // );
    m_chooser.setDefaultOption("Shoot Auto", m_ShootAuto);
    // m_chooser.addOption("Turn and Shoot Auto", m_TurnShootAuto);

    SmartDashboard.putData(m_chooser);
    // m_IntakeArmSubsystem.setDefaultCommand(m_IntakeArmSubsystem.loadPosition());
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
    // Turn to 180 degrees when the 'X' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kA.value)
    // .onTrue(new TurnToAngleCommand(() -> 180, m_robotDrive).withTimeout(3));

    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new DropShooterAngleCommand(m_ShooterSubsystem, false));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new TurnToAngleCommand(() -> -135, m_DriveSubsystem).withTimeout(3));

    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new TurnToAngleCommand(() -> 135, m_DriveSubsystem).withTimeout(3));

    // Turn to 0 degrees when the 'B' button is pressed, with a 3 second timeout
    // new JoystickButton(m_driverController, Button.kY.value)
    // .onTrue(new TurnToAngleCommand(() -> 0, m_robotDrive).withTimeout(3));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new speedAdjustCommand(m_DriveSubsystem, true));

    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //     .onTrue(new speedAdjustCommand(m_robotDrive, false));

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new RestartGyroCommand(m_DriveSubsystem));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_DriveSubsystem.drive(m_DriveSubsystem
        .BuildFieldTrajectory(new Pose2d(1.778, 0, Rotation2d.fromDegrees(180))))));


    // Activates Shooter for 3 seconds. hopefully.
    new JoystickButton(m_OperatorController, Button.kY.value)
        .onTrue(new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(3));

    new POVButton(m_OperatorController, 0)
        .onTrue(new DropShooterAngleCommand(m_ShooterSubsystem, true));

    new POVButton(m_OperatorController, 180)
        .onTrue(new DropShooterAngleCommand(m_ShooterSubsystem, false));

    new JoystickButton(m_OperatorController, Button.kRightBumper.value)
        .whileTrue(new ActivateIntakeCommand(m_IntakeSubsystem, -0.45)); // Negative = ingest note

    // Manual Overrides for stick control of intake arm and climber
    new JoystickButton(m_OperatorController, Button.kLeftBumper.value)
        .whileTrue(new ManualOverrideCommand(m_IntakeArmSubsystem, m_ClimberSubsystem, m_OperatorController,
            m_IntakeSubsystem));

    new JoystickButton(m_OperatorController, Button.kA.value)
        .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, -0.3).withTimeout(1));

    new JoystickButton(m_OperatorController, Button.kB.value)
        .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, 0.5).withTimeout(1));

    new JoystickButton(m_OperatorController, Button.kX.value)
        .onTrue(new ActivateIntakeCommand(m_IntakeSubsystem, 0.8).withTimeout(1));

    new POVButton(m_OperatorController, 90)
        .onTrue(new TurnToAngleCommand(() -> VisionUtils.calculateNoteAngle(m_gyro), m_DriveSubsystem).withTimeout(3));

    new POVButton(m_OperatorController, 270)
        .onTrue(new TurnToAngleCommand(() -> VisionUtils.calculateNoteAngle(m_gyro), m_DriveSubsystem).withTimeout(3));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory toFirstNote = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0,0,new Rotation2d(0)),
    // List.of(
    // new Translation2d(1,0),
    // new Translation2d(1,1)
    // ),
    // new Pose2d(2,1, Rotation2d.fromDegrees(0)),
    // trajectoryConfig);
    // Trajectory toSpeakerFromFirstNote = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(2,1,new Rotation2d(0)),
    // List.of(
    // new Translation2d(1,1),
    // new Translation2d(1,0)
    // ),
    // new Pose2d(0,0, Rotation2d.fromDegrees(0)),
    // trajectoryConfig);

    // PIDController xController = new PIDController(AutoConstants.kPXController, 0,
    // 0);
    // PIDController yController = new PIDController(AutoConstants.kPYController, 0,
    // 0);
    // ProfiledPIDController theteController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // theteController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand toFirstNoteControllerCommand = new
    // SwerveControllerCommand(
    // toFirstNote,
    // m_robotDrive::getPose,
    // DriveConstants.kDriveKinematics,
    // xController,
    // yController,
    // theteController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // SwerveControllerCommand toSpeakerFromFirstControllerCommand = new
    // SwerveControllerCommand(
    // toSpeakerFromFirstNote,
    // m_robotDrive::getPose,
    // DriveConstants.kDriveKinematics,
    // xController,
    // yController,
    // theteController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // return new SequentialCommandGroup(
    // new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1),
    // new InstantCommand(() ->
    // m_robotDrive.resetOdometry(toFirstNote.getInitialPose())),
    // toFirstNoteControllerCommand,

    // toSpeakerFromFirstControllerCommand,
    // new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1)
    // );

    // Command driveToFirstNoteCommand = m_robotDrive.drive(firstNoteTrajectory);
    Pose2d positionOfFirstNote = new Pose2d(1.778, 0, Rotation2d.fromDegrees(0));
    Pose2d positionOfSecondNote = new Pose2d(1.778, 1.397, Rotation2d.fromDegrees(0));
    Pose2d positionOfThirdNote = new Pose2d(1.778, -1.397, Rotation2d.fromDegrees(0));
    Pose2d positionOfHome = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    
    Trajectory toFirst = m_DriveSubsystem.BuildFieldTrajectory(positionOfFirstNote);
    Trajectory toSecond = m_DriveSubsystem.BuildFieldTrajectory(positionOfSecondNote);
    Trajectory toThird = m_DriveSubsystem.BuildFieldTrajectory(positionOfThirdNote);
    Trajectory toHome = m_DriveSubsystem.BuildFieldTrajectory(positionOfHome);

    Command goToFirst = m_DriveSubsystem.drive(toFirst);
    Command goToSecond = m_DriveSubsystem.drive(toSecond);
    Command goToThird = m_DriveSubsystem.drive(toThird);
    Command goToHome = m_DriveSubsystem.drive(toHome);
    
    Command goToHome2 = m_DriveSubsystem.drive(toHome);
    
    Command goToHome3 = m_DriveSubsystem.drive(toHome);

    return new SequentialCommandGroup(
       // new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1),
       // new InstantCommand(() -> m_robotDrive.resetOdometry(m_robotDrive.getPose())),
        goToFirst
        // goToHome,
        // goToSecond,
        // goToHome2,
        // goToThird,
        // goToHome3,
        // new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1));

    // return m_chooser.getSelected();
    );
  }
}