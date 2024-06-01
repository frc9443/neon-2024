// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake_arm.*;
import frc.robot.subsystems.pneumatics.*;
import frc.robot.subsystems.shooter.*;
import frc.utils.OffsetGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private DriveSubsystem m_DriveSubsystem;
        private Shooter shooter;
        private Pneumatics pneumatics;
        private Intake intake;
        private IntakeArm intakeArm;
        private Climber climber;
        private BlinkinSubsystem m_BlinkinSubsystem;
        private VisionSubsystem m_VisionSubsystem;

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
        // XboxController m_ColorController = new
        // XboxController(OIConstants.kColorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                OffsetGyro m_gyro;

                switch (Constants.getRobot()) {
                        case NEON -> {
                                climber = new Climber(new ClimberIOSparkMax());
                                intake = new Intake(new IntakeIOSparkMax());
                                intakeArm = new IntakeArm(new IntakeArmIOSparkMax());
                                shooter = new Shooter(new ShooterIOSparkFlex());
                                m_gyro = new OffsetGyro(new AHRS(SerialPort.Port.kUSB));
                                pneumatics = new Pneumatics(new PneumaticsIORev());
                        }
                        case HELIUM -> {
                                m_gyro = new OffsetGyro(new AHRS(SPI.Port.kMXP));
                        }
                        case SIM -> {
                                climber = new Climber(new ClimberIOSim());
                                intake = new Intake(new IntakeIOSim());
                                intakeArm = new IntakeArm(new IntakeArmIOSim());
                                shooter = new Shooter(new ShooterIOSim());
                                m_gyro = new OffsetGyro(new AHRS(SerialPort.Port.kUSB));
                                pneumatics = new Pneumatics(new PneumaticsIOSim());
                        }
                        default -> throw new RuntimeException(
                                        "I don't know how to configure " + Constants.getRobot().toString());
                }

                // No-op subsystem implementations (if not configured above)
                if (climber == null) {
                        climber = new Climber(new ClimberIO() {});
                }
                if (intake == null) {
                        intake = new Intake(new IntakeIO() {});
                }
                if (intakeArm == null) {
                        intakeArm = new IntakeArm(new IntakeArmIO() {});
                }
                if (shooter == null) {
                        shooter = new Shooter(new ShooterIO() {});
                }
                if (pneumatics == null) {
                        pneumatics = new Pneumatics(new PneumaticsIO() {});
                }

                m_DriveSubsystem = new DriveSubsystem(m_gyro);
                m_VisionSubsystem = new VisionSubsystem();
                m_BlinkinSubsystem = new BlinkinSubsystem(m_VisionSubsystem, intake, intakeArm);
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_DriveSubsystem.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_DriveSubsystem.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true,
                                                                m_driverController.getLeftBumper() ? 4.8 : 3.0),
                                                m_DriveSubsystem));

                // Register Named Commands
                NamedCommands.registerCommand("ShootCommand",
                                new ShootCommand(shooter, intake, pneumatics));

                NamedCommands.registerCommand("EnsurePressureCommand",
                                new EnsurePressureCommand(pneumatics));

                NamedCommands.registerCommand("IntakeInCommand", intakeArm.load());

                NamedCommands.registerCommand("IntakeOutCommand", intakeArm.intake());

                NamedCommands.registerCommand("ActivateIntakeCommand",
                                new ActivateIntakeCommand(intake).withTimeout(2));

                NamedCommands.registerCommand("RaiseShooterAngleCommand",
                                new ChangeShooterAngleCommand(pneumatics, false));

                NamedCommands.registerCommand("DropShooterAngleCommand",
                                new ChangeShooterAngleCommand(pneumatics, true));

                NamedCommands.registerCommand("DetectNoteCommand",
                                new AutoLimeLightTargetCommand(m_DriveSubsystem, intake).withTimeout(2));

                NamedCommands.registerCommand("SpeakerAimCommand",
                                new TurnToAprilTagCommand(m_DriveSubsystem, m_VisionSubsystem)
                                                .withTimeout(.8));

                m_chooser.setDefaultOption("Shoot Auto", new PathPlannerAuto("Shoot Auto"));
                m_chooser.addOption("4 Note Auto", new PathPlannerAuto("4 Note Auto"));
                m_chooser.addOption("Defensive Auto", new PathPlannerAuto("Defensive Auto"));
                m_chooser.addOption("Amp Side Auto", new PathPlannerAuto("Amp Side Auto"));
                m_chooser.addOption("Source Side Mid Auto", new PathPlannerAuto("Source Side Mid Auto"));
                m_chooser.addOption("4 Note, No Amp Note", new PathPlannerAuto("4 Note Auto No Amp"));
                SmartDashboard.putData(m_chooser);
                intakeArm.setDefaultCommand(intakeArm.load());
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

                new JoystickButton(m_driverController, Button.kX.value)
                                .whileTrue(new TurnToAprilTagCommand(m_DriveSubsystem, m_VisionSubsystem));

                new JoystickButton(m_driverController, Button.kA.value)
                                .onTrue(new EjectCommand(shooter, intake).withTimeout(1));

                new JoystickButton(m_driverController, Button.kB.value)
                                .whileTrue(new AutoLimeLightTargetCommand(m_DriveSubsystem, intake));

                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .onTrue(new speedAdjustCommand(m_DriveSubsystem, true));

                new JoystickButton(m_driverController, Button.kY.value)
                                .onTrue(new RestartGyroCommand(m_DriveSubsystem));

                // Activates Shooter for 3 seconds.
                new JoystickButton(m_OperatorController, Button.kY.value)
                                .onTrue(new ShootCommand(shooter, intake, pneumatics).withTimeout(1));

                new POVButton(m_OperatorController, 0)
                                .onTrue(new ChangeShooterAngleCommand(pneumatics, false));

                new POVButton(m_OperatorController, 180)
                                .onTrue(new ChangeShooterAngleCommand(pneumatics, true));

                switch (Constants.controlMode) {
                        case Neon_2024_PostSeason -> {
                                new JoystickButton(m_OperatorController, Button.kRightBumper.value)
                                                .whileTrue(new DoIntakeCommand(intake, intakeArm));
                        }
                        default -> {
                                new JoystickButton(m_OperatorController, Button.kRightBumper.value)
                                                .whileTrue(new ActivateIntakeCommand(intake).withTimeout(3));
                        }
                }

                // Manual Overrides for stick control of intake arm and climber
                new JoystickButton(m_OperatorController, Button.kLeftBumper.value)
                                .whileTrue(new ManualOverrideCommand(intakeArm, climber,
                                                m_OperatorController,
                                                intake));

                new JoystickButton(m_OperatorController, Button.kB.value)
                                .onTrue(new AmpShootCommand(intake));

                new JoystickButton(m_OperatorController, Button.kA.value)
                                .onTrue(intakeArm.amp());

                if (Constants.controlMode == Constants.ControlMode.Neon_2024_Competition) {
                        new POVButton(m_OperatorController, 90)
                                        .onTrue(intakeArm.load());

                        new POVButton(m_OperatorController, 270)
                                        .onTrue(intakeArm.intake());
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}