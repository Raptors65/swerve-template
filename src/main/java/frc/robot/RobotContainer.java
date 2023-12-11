
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopRoutines.RotateToAngle;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveSlow;
import frc.robot.commands.swerve.ToggleFieldOriented;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  // The robot's subsystems
  public final DriveSubsystem m_drive;

  public FieldSim m_fieldSim;

  // The driver, codriver and arm controllers

  public CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  public CommandXboxController m_armsController = new CommandXboxController(
      OIConstants.kArmControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_drive = new DriveSubsystem();

    Pref.deleteUnused();

    Pref.addMissing();

    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    LiveWindow.disableAllTelemetry();
    m_fieldSim = new FieldSim(m_drive);

    m_fieldSim.initSim();

    setDefaultCommands();

    configDriverButtons();

    configArmsButtons();

  }

  private void setDefaultCommands() {

    m_drive.setDefaultCommand(getDriveCommand());

  }

  private void configDriverButtons() {

    m_driverController.leftTrigger()
        .whileTrue(getSlowDriveCommand());

    // m_driverController.leftBumper().whileTrue(new EjectPieceFromIntake(m_intake,
    // 5));

    m_driverController.x().onTrue(new ToggleFieldOriented(m_drive));

    m_driverController.back()
        .onTrue(new InstantCommand(() -> m_drive.resetGyro()));
    
    m_driverController.rightTrigger()
      .onTrue(getRotateCommand(0));

    // m_driverController.back()

  }

  private void configArmsButtons() {

    m_armsController.rightTrigger().onTrue(Commands.runOnce(() -> m_drive.clearFaults()));

    // m_armsController.povUp().onTrue(new TurnToGamepiece(m_drive, 2, true));//
    // .withTimeout(3));

    // m_armsController.back() DO NOT ASSIGN ALREADY USED IN JOG COMMANDS TO
    // OVERRIDE SOFTWARE LIMITS

  }

  public Command getDriveCommand() {
    return new SetSwerveDrive(m_drive,
        () -> m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0),
        () -> m_driverController.getRawAxis(4));

  }

  public Command getSlowDriveCommand() {
    return new SetSwerveDriveSlow(m_drive,
        () -> m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0),
        () -> m_driverController.getRawAxis(4));

  }

  public Command getStopDriveCommand() {
    return new InstantCommand(() -> m_drive.stopModules());
  }

  public Command getRotateCommand(double angle) {

    return new RotateToAngle(m_drive, angle);
  }

  public void simulationPeriodic() {

    m_fieldSim.periodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
    // m_pt.update();

  }

}
