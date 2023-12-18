// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleTuneConstants;
import frc.robot.Constants.SYSIDConstants;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;

public class SwerveModuleSM extends SubsystemBase {

  public final CANSparkMax m_driveMotor;

  public final CANSparkMax m_turnMotor;

  public final RelativeEncoder m_driveEncoder;

  public final RelativeEncoder m_turnEncoder;

  public final int m_locationIndex;

  private final PIDController m_driveVelController = new PIDController(ModuleTuneConstants.kPModuleDriveController,
      ModuleTuneConstants.kIModuleDriveController, ModuleTuneConstants.kDModuleDriveController);

  private PIDController m_turnPosController = new PIDController(ModuleTuneConstants.kPModuleTurningController,
      ModuleTuneConstants.kIModuleTurningController, ModuleTuneConstants.kDModuleTurningController);

  public final CANCoder m_turnCANcoder;

  SwerveModuleState state = new SwerveModuleState();

  public int m_moduleNumber;

  public String[] modAbrev = { "FL ", "FR ", "RL ", "RR " };

  String driveLayout;

  String turnLayout;

  String canCoderLayout;

  Pose2d m_pose;

  double testAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      SYSIDConstants.ksDriveVoltSecondsPerMeter,
      SYSIDConstants.kvDriveVoltSecondsSquaredPerMeter,
      SYSIDConstants.kaDriveVoltSecondsSquaredPerMeter);

  private double m_lastAngle;
  public double angle;

  public double m_turnEncoderOffset;

  private double tolDegPerSec = .01;
  private double toleranceDeg = .25;
  public boolean driveMotorConnected;
  public boolean turnMotorConnected;
  public boolean turnCoderConnected;

  public SendableBuilder m_builder;

  public boolean m_isOpenLoop;

  public boolean driveBrakeMode;

  public boolean turnBrakeMode;

  public boolean isRotating;

  private double simAngle;

  private int moduleFaultSeen;

  private int moduleStickyFaultSeen;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset
   */
  public SwerveModuleSM(
      int locationIndex,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double turningEncoderOffset) {

    m_locationIndex = locationIndex;

    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);

    m_turnMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);

    m_turnMotor.restoreFactoryDefaults();

    m_driveMotor.restoreFactoryDefaults();

    m_turnMotor.setSmartCurrentLimit(CurrentLimitConstants.turnMotorSmartLimit);

    m_driveMotor.setSmartCurrentLimit(CurrentLimitConstants.driveMotorSmartLimit);

    m_driveMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);

    m_turnMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);

    // absolute encoder used to establish known wheel position on start position
    m_turnCANcoder = new CANCoder(cancoderCanChannel, "CV1");
    m_turnCANcoder.configFactoryDefault();
    m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());
    m_turnEncoderOffset = turningEncoderOffset;

    m_driveMotor.setInverted(driveMotorReversed);

    m_turnMotor.setInverted(turningMotorReversed);

    // Set neutral mode to brake
    m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);

    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerEncRev
        / 60);

    m_turnEncoder = m_turnMotor.getEncoder();

    m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);

    m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

    m_turnPosController.enableContinuousInput(-180, 180);

    // checkCAN();

    setCANTimes();

    resetAngleToAbsolute();

    if (RobotBase.isSimulation()) {

      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, 3, 5600);

      m_driveEncoder.setPositionConversionFactor(1);

      m_driveEncoder.setVelocityConversionFactor(1);

      REVPhysicsSim.getInstance().addSparkMax(m_turnMotor, 3, 5600);

      m_turnEncoder.setPositionConversionFactor(1);

      m_turnEncoder.setVelocityConversionFactor(1);

      m_turnPosController.setP(.01);

      m_driveVelController.setP(.25);

    }

  }

  @Override
  public void periodic() {

    if (moduleFaultSeen == 0)
      moduleFaultSeen = getFaults();

    if (moduleStickyFaultSeen == 0)
      moduleStickyFaultSeen = getStickyFaults();

    // if (Pref.getPref("SwerveTune") == 1 && tuneOn == 0) {

    // tuneOn = 1;

    // tunePosGains();

    // tuneDriveVelGains();
    // }

    // if (tuneOn == 1) {

    // tuneOn = (int) Pref.getPref("SwerveTune");
    // }

    // if (m_turnCANcoder.getFaults(null)) {
    // // SmartDashboard.putStringArray("CanCoderFault"
    // // + m_modulePosition.toString(), m_turnCANcoder.getFaults());
    // // SmartDashboard.putStringArray("CanCoderStickyFault"
    // // + String.valueOf(m_locationIndex), m_turnCANcoder.getStickyFaults());

    // }
  }

  @Override
  public void simulationPeriodic() {

    REVPhysicsSim.getInstance().run();

    double diff = angle - simAngle;

    simAngle += diff / 10;

  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees((getTurnAngleDegs())));
  }

  public double[] getMeasuredState() {
    double[] temp = { 0, 0 };
    temp[0] = getState().angle.getRadians();
    temp[1] = getState().speedMetersPerSecond;
    return temp;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getHeadingRotation2d());
  }

  public double[] getDesiredState() {
    double[] temp = { 0, 0 };
    if (state != null) {
      temp[0] = state.angle.getRadians();
      temp[1] = state.speedMetersPerSecond;
    }
    return temp;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

    // SmartDashboard.putNumber("StateSpeed", state.speedMetersPerSecond);

    // SmartDashboard.putNumber("StateDegs", state.angle.getDegrees());

    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    if ((Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01)))

      angle = m_lastAngle;

    else

      angle = state.angle.getDegrees();

    m_lastAngle = angle;

    if (m_isOpenLoop) {

      driveMotorMoveOpenLoop(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);

    } else {

      driveMotorMoveVelocity(state.speedMetersPerSecond);
    }

    positionTurn(angle);

  }

  public double getDriveSpeedSetpoint() {

    return state.speedMetersPerSecond;

  }

  public void driveMotorMoveOpenLoop(double speed) {

    m_driveMotor.setVoltage(RobotController.getBatteryVoltage() * speed);

  }

  public void driveMotorMoveVelocity(double speedMPS) {

    double ff = (12 * speedMPS) * .15 / DriveConstants.kMaxSpeedMetersPerSecond;

    if (RobotBase.isReal())

      ff = feedforward.calculate(speedMPS);

    m_driveMotor

        .setVoltage(ff + m_driveVelController.calculate(getDriveVelocity()));

  }

  public void positionTurn(double angle) {

    double pidOut = m_turnPosController.calculate(getTurnAngleDegs(), angle);

    // SmartDashboard.putNumber("PIDOUT", pidOut);

    // SmartDashboard.putNumber("ATAE", turnAngleError);

    // SmartDashboard.putNumber("TEST", tst++);

    // if robot is not moving, stop the turn motor oscillating
    // if (turnAngleError < turnDeadband

    // && Math.abs(state.speedMetersPerSecond) <=
    // (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))

    // pidOut = 0;

    m_turnMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());

  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(0);
  }

  public double getHeadingDegs() {
    return getTurnAngleDegs();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegs());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public void setDriveBrakeMode(boolean on) {
    driveBrakeMode = on;
    if (on)
      m_driveMotor.setIdleMode(IdleMode.kBrake);
    else
      m_driveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean on) {
    turnBrakeMode = on;
    if (on) {
      m_turnMotor.setIdleMode(IdleMode.kBrake);

    } else
      m_turnMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetAngleToAbsolute() {
    double angle = 0;
    if (RobotBase.isReal())
      angle = m_turnCANcoder.getAbsolutePosition() - m_turnEncoderOffset;
    m_turnEncoder.setPosition(angle);
  }

  public double getTurnAngleDegs() {
    if (RobotBase.isReal())
      return m_turnEncoder.getPosition();
    else
      return simAngle;
    // return m_turnEncoder.getPosition() *
    // ModuleConstants.kTurningDegreesPerEncRev;

  }

  public void turnMotorMove(double speed) {

    m_turnMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public double getDriveVelocity() {
    if (RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return (m_driveEncoder.getVelocity() * ModuleConstants.kDriveMetersPerEncRev) / 60;

  }

  public double getDrivePosition() {
    if (RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
      return m_driveEncoder.getPosition() * ModuleConstants.kDriveMetersPerEncRev;

  }

  public double getDriveAppliedOutput() {
    if (RobotBase.isReal())
      return m_driveMotor.getAppliedOutput();
    else
      return 0;
  }

  public double getDriveCurrent() {
    if (RobotBase.isReal())
      return m_driveMotor.getOutputCurrent();
    else
      return 0;
  }

  public double getTurnVelocity() {
    if (RobotBase.isReal())
      return m_turnEncoder.getVelocity();
    else
      return 0;// (m_turnEncoder.getVelocity() * ModuleConstants.kTurningDegreesPerEncRev) /
               // 60;

  }

  public double getTurnCurrent() {
    if (RobotBase.isReal())
      return m_turnMotor.getOutputCurrent();
    else
      return 0;
  }

  public double getTurnAppliedOutput() {
    if (RobotBase.isReal())
      return m_turnMotor.getAppliedOutput();
    else
      return 0;
  }

  public boolean turnInPosition(double targetAngle) {
    return Math.abs(targetAngle - getTurnAngleDegs()) < toleranceDeg;
  }

  public boolean turnIsStopped() {
    if (RobotBase.isReal())
      return Math.abs(m_turnEncoder.getVelocity()) < tolDegPerSec;
    else
      return false;
  }

  public int getFaults() {
    return m_driveMotor.getFaults() + m_turnMotor.getFaults();// || m_turnCANcoder.getFaulted();
  }

  public int getStickyFaults() {
    return m_driveMotor.getStickyFaults() + m_turnMotor.getStickyFaults();
  }

  public void clearFaults() {
    m_driveMotor.clearFaults();
    m_turnMotor.clearFaults();
    m_turnCANcoder.clearStickyFaults();
    moduleFaultSeen = 0;
    moduleStickyFaultSeen = 0;

  }

  public boolean checkCAN() {
    driveMotorConnected = m_driveMotor.getFirmwareVersion() != 0;
    turnMotorConnected = m_turnMotor.getFirmwareVersion() != 0;
    turnCoderConnected = m_turnCANcoder.getFirmwareVersion() > 0;
    return RobotBase.isSimulation() || (driveMotorConnected && turnMotorConnected && turnCoderConnected);
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }

  public void tunePosGains() {
    m_turnPosController.setP(Pref.getPref("SwerveTurnPoskP"));
    m_turnPosController.setI(Pref.getPref("SwerveTurnPoskI"));
    m_turnPosController.setD(Pref.getPref("SwerveTurnPoskD"));
    // m_turnController.setIZone(Pref.getPref("SwerveTurnPoskIz"));
  }

  public void tuneDriveVelGains() {
    m_driveVelController.setP(Pref.getPref("SwerveVelkP"));
    m_driveVelController.setI(Pref.getPref("SwerveVelkI"));
    m_driveVelController.setD(Pref.getPref("SwerveVelkD"));

  }

  private void setCANTimes() {

    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    // Set neutral mode to brake
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);

  }

}