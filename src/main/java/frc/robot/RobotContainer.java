// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake_arm.*;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.pneumatics.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.utils.OffsetGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
        private Leds leds;
        private Vision vision;

        // A chooser for autonomous commands
        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

        // The driver's controller
        CommandXboxController driver = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController operator = new CommandXboxController(OIConstants.kOperatorControllerPort);
        // XboxController m_ColorController = new
        // XboxController(OIConstants.kColorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                OffsetGyro m_gyro = null;

                switch (Constants.getRobot()) {
                        case NEON -> {
                                climber = new Climber(new ClimberIOSparkMax());
                                intake = new Intake(new IntakeIOSparkMax());
                                intakeArm = new IntakeArm(new IntakeArmIOSparkMax());
                                shooter = new Shooter(new ShooterIOSparkFlex());
                                m_gyro = new OffsetGyro(new AHRS(SerialPort.Port.kUSB));
                                pneumatics = new Pneumatics(new PneumaticsIORev());
                                leds = new Leds(new LedsIOBlinkin());
                                vision = new Vision(new VisionIOPhoton());
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
                        default -> {
                                // Just use the No-op implementations
                        }
                }

                // No-op subsystem implementations (if not configured above)
                if (climber == null) {
                        climber = new Climber(new ClimberIO() {
                        });
                }
                if (intake == null) {
                        intake = new Intake(new IntakeIO() {
                        });
                }
                if (intakeArm == null) {
                        intakeArm = new IntakeArm(new IntakeArmIO() {
                        });
                }
                if (shooter == null) {
                        shooter = new Shooter(new ShooterIO() {
                        });
                }
                if (pneumatics == null) {
                        pneumatics = new Pneumatics(new PneumaticsIO() {
                        });
                }
                if (leds == null) {
                        leds = new Leds(new LedsIO() {
                        });
                }
                if (vision == null) {
                        vision = new Vision(new VisionIO() {
                        });
                }

                m_DriveSubsystem = new DriveSubsystem(m_gyro);

                // IoC
                leds.setIntake(intake);
                leds.setIntakeArm(intakeArm);
                leds.setVision(vision);

                configureButtonBindings();
                configureAutos();

                // Initialize intakeArm to the loaded position
                intakeArm.setDefaultCommand(intakeArm.load());
                intake.setDefaultCommand(intake.run(() -> intake.stop()));
        }

        private void configureAutos() {
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
                                new TurnToAprilTagCommand(m_DriveSubsystem, vision)
                                                .withTimeout(.8));

                // Configure the auto command chooser
                // TODO: consider using AutoBuilder.buildAutoChooser() instead of manual setup
                autoChooser.addDefaultOption("Shoot Auto", new PathPlannerAuto("Shoot Auto"));
                autoChooser.addOption("4 Note Auto", new PathPlannerAuto("4 Note Auto"));
                autoChooser.addOption("Defensive Auto", new PathPlannerAuto("Defensive Auto"));
                autoChooser.addOption("Amp Side Auto", new PathPlannerAuto("Amp Side Auto"));
                autoChooser.addOption("Source Side Mid Auto", new PathPlannerAuto("Source Side Mid Auto"));
                autoChooser.addOption("4 Note, No Amp Note", new PathPlannerAuto("4 Note Auto No Amp"));
        }

        private void configureButtonBindings() {

                // Driver Controls
                driver.x().whileTrue(new TurnToAprilTagCommand(m_DriveSubsystem, vision));
                driver.a().onTrue(new EjectCommand(shooter, intake).withTimeout(1));
                driver.b().whileTrue(new AutoLimeLightTargetCommand(m_DriveSubsystem, intake));
                driver.y().onTrue(new RestartGyroCommand(m_DriveSubsystem));
                driver.rightBumper().onTrue(new speedAdjustCommand(m_DriveSubsystem, true));

                m_DriveSubsystem.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                m_DriveSubsystem.run(
                                                () -> m_DriveSubsystem.drive(
                                                                -MathUtil.applyDeadband(driver.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driver.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driver.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true,
                                                                driver.getHID().getLeftBumper() ? 4.8 : 3.0))
                                                .withName("Drive Teleop"));

                // Operator Controls
                operator.a().onTrue(intakeArm.amp());
                operator.b().onTrue(new AmpShootCommand(intake));
                operator.y().onTrue(new ShootCommand(shooter, intake, pneumatics).withTimeout(1));
                operator.povUp().onTrue(new ChangeShooterAngleCommand(pneumatics, false));
                operator.povDown().onTrue(new ChangeShooterAngleCommand(pneumatics, true));

                // Manual Overrides for stick control of intake arm and climber
                operator.leftBumper().whileTrue(new ManualOverrideCommand(intake, intakeArm, climber, operator));

                switch (Constants.controlMode) {
                        case Neon_2024_PostSeason -> {
                                operator.rightBumper().whileTrue(new DoIntakeCommand(intake, intakeArm));
                        }
                        default -> {
                                operator.rightBumper().whileTrue(new ActivateIntakeCommand(intake).withTimeout(3));
                        }
                }

                if (Constants.controlMode == Constants.ControlMode.Neon_2024_Competition) {
                        operator.povLeft().onTrue(intakeArm.load());
                        operator.povRight().onTrue(intakeArm.intake());
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}