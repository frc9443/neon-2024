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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.intake_arm.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake_arm.IntakeArmIOSim;
import frc.robot.subsystems.intake_arm.IntakeArmIOSparkMax;
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
        private ShooterSubsystem m_ShooterSubsystem;
        private CompressorSubsystem m_CompressorSubsystem;
        private IntakeSubsystem m_IntakeSubsystem;
        private IntakeArm intakeArm;
        private ClimberSubsystem m_ClimberSubsystem;
        private BlinkinSubsystem m_BlinkinSubsystem;
        private VisionSubsystem m_VisionSubsystem;

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
        //XboxController m_ColorController = new XboxController(OIConstants.kColorControllerPort);
        private final OffsetGyro m_gyro;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                switch (Constants.getRobot()) {
                        case NEON -> {
                                intakeArm = new IntakeArm(new IntakeArmIOSparkMax());
                                m_gyro = new OffsetGyro(new AHRS(SerialPort.Port.kUSB));
                        }
                        case HELIUM -> {
                                m_gyro = new OffsetGyro(new AHRS(SPI.Port.kMXP));
                        }
                        case SIM -> {
                                intakeArm = new IntakeArm(new IntakeArmIOSim());
                                m_gyro = new OffsetGyro(new AHRS(SerialPort.Port.kUSB));
                        }
                        default -> throw new RuntimeException("I don't know how to configure " + Constants.getRobot().toString());
                }

                m_DriveSubsystem = new DriveSubsystem(m_gyro);
                m_ShooterSubsystem = new ShooterSubsystem();
                m_CompressorSubsystem = new CompressorSubsystem();
                m_IntakeSubsystem = new IntakeSubsystem();
                m_ClimberSubsystem = new ClimberSubsystem();
                m_VisionSubsystem = new VisionSubsystem();
                m_BlinkinSubsystem = new BlinkinSubsystem(m_VisionSubsystem, m_IntakeSubsystem, intakeArm);
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
                                                                true, true,m_driverController.getLeftBumper() ? 4.8 : 3.0),
                                                m_DriveSubsystem));

                // Register Named Commands
                NamedCommands.registerCommand("ShootCommand",
                                new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem));
               
                NamedCommands.registerCommand("EnsurePressureCommand",
                                new EnsurePressureCommand(m_CompressorSubsystem));

                NamedCommands.registerCommand("IntakeInCommand", intakeArm.load());

                NamedCommands.registerCommand("IntakeOutCommand", intakeArm.intake());

                NamedCommands.registerCommand("ActivateIntakeCommand",
                                new ActivateIntakeCommand(m_IntakeSubsystem).withTimeout(2));

                NamedCommands.registerCommand("RaiseShooterAngleCommand",
                                new ChangeShooterAngleCommand(m_ShooterSubsystem, false));

                NamedCommands.registerCommand("DropShooterAngleCommand",
                                new ChangeShooterAngleCommand(m_ShooterSubsystem, true));

                NamedCommands.registerCommand("DetectNoteCommand",
                                new AutoLimeLightTargetCommand(m_DriveSubsystem , m_IntakeSubsystem).withTimeout(2));

                NamedCommands.registerCommand("SpeakerAimCommand",
                                new TurnToAprilTagCommand(m_DriveSubsystem, m_VisionSubsystem, m_ShooterSubsystem).withTimeout(.8));

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
                                .whileTrue(new TurnToAprilTagCommand(m_DriveSubsystem, m_VisionSubsystem, m_ShooterSubsystem));

                new JoystickButton(m_driverController, Button.kA.value)
                                .onTrue(new EjectCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1));

                new JoystickButton(m_driverController, Button.kB.value)
                                .whileTrue(new AutoLimeLightTargetCommand(m_DriveSubsystem, m_IntakeSubsystem));

                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .onTrue(new speedAdjustCommand(m_DriveSubsystem, true));

                new JoystickButton(m_driverController, Button.kY.value)
                                .onTrue(new RestartGyroCommand(m_DriveSubsystem));


                // Activates Shooter for 3 seconds.
                new JoystickButton(m_OperatorController, Button.kY.value)
                                .onTrue(new ShootCommand(m_ShooterSubsystem, m_IntakeSubsystem).withTimeout(1));

                new POVButton(m_OperatorController, 0)
                                .onTrue(new ChangeShooterAngleCommand(m_ShooterSubsystem, false));

                new POVButton(m_OperatorController, 180)
                                .onTrue(new ChangeShooterAngleCommand(m_ShooterSubsystem, true));

                switch (Constants.controlMode) {
                        case Neon_2024_PostSeason -> {
                                new JoystickButton(m_OperatorController, Button.kRightBumper.value)
                                        .whileTrue(new DoIntakeCommand(m_IntakeSubsystem, intakeArm));
                        }
                        default -> {
                                new JoystickButton(m_OperatorController, Button.kRightBumper.value)
                                        .whileTrue(new ActivateIntakeCommand(m_IntakeSubsystem).withTimeout(3));
                        }
                }

                // Manual Overrides for stick control of intake arm and climber
                new JoystickButton(m_OperatorController, Button.kLeftBumper.value)
                                .whileTrue(new ManualOverrideCommand(intakeArm, m_ClimberSubsystem,
                                                m_OperatorController,
                                                m_IntakeSubsystem));

                new JoystickButton(m_OperatorController, Button.kB.value)
                        .onTrue(new AmpShootCommand(m_IntakeSubsystem));

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