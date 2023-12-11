// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.Pref;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics m_kinematics = DriveConstants.m_kinematics;

  public boolean isOpenLoop = true;// RobotBase.isSimulation() && !DriverStation.isAutonomousEnabled();

  final PowerDistribution m_pdp = new PowerDistribution();

  public final SwerveModuleSM m_frontLeft = new SwerveModuleSM(
      IDConstants.FRONT_LEFT_LOCATION,
      CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed,
      PDPConstants.FRONT_LEFT_DRIVE_CHANNEL,
      PDPConstants.FRONT_LEFT_TURN_CHANNEL,
      CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_frontRight = new SwerveModuleSM(
      IDConstants.FRONT_RIGHT_LOCATION,
      CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningMotorReversed,
      PDPConstants.FRONT_RIGHT_DRIVE_CHANNEL,
      PDPConstants.FRONT_RIGHT_TURN_CHANNEL,
      CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_backLeft = new SwerveModuleSM(
      IDConstants.REAR_LEFT_LOCATION,
      CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
      CanConstants.BACK_LEFT_MODULE_STEER_CANCODER,
      DriveConstants.kBackLeftDriveMotorReversed,
      DriveConstants.kBackLeftTurningMotorReversed,
      PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
      PDPConstants.BACK_LEFT_TURN_CHANNEL,
      CanConstants.BACK_LEFT_MODULE_STEER_OFFSET);

  public final SwerveModuleSM m_backRight = new SwerveModuleSM(
      IDConstants.REAR_RIGHT_LOCATION,
      CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
      CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER,
      DriveConstants.kBackRightDriveMotorReversed,
      DriveConstants.kBackRightTurningMotorReversed,
      PDPConstants.BACK_LEFT_DRIVE_CHANNEL,
      PDPConstants.BACK_LEFT_TURN_CHANNEL,
      CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

  // The gyro sensor

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 100);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public SimDouble m_simAngle;// navx sim

  public double targetAngle;

  public boolean m_fieldOriented = false;

  public PIDController rotatePID = new PIDController(DriveConstants.kTurnP,
      DriveConstants.kTurnI, DriveConstants.kTurnD);

  public PIDController xPID = new PIDController(
      PPConstants.kPXController, PPConstants.kIXController, PPConstants.kIXController); // X

  public PIDController yPID = new PIDController(PPConstants.kPYController, PPConstants.kIYController,
      PPConstants.kDYController);

  public PIDController thetaPID = new PIDController(PPConstants.kPThetaController, PPConstants.kIThetaController,
      PPConstants.kDThetaController);

  public boolean isRotating;

  public boolean trajectoryRunning;

  public boolean ALL_CANOK;

  private final double fieldOrientOffset = 0;

  public float gyroStartPitch;

  public int moduleFaultSeen;

  public int moduleStickyFaultSeen;

  public double[] measuredStates = { 0, 0, 0, 0, 0, 0, 0, 0 };

  public double[] desiredStates = { 0, 0, 0, 0, 0, 0, 0, 0 };

  public double yTrajStart;

  public Command autonomousCommand = new DoNothing();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_gyro.reset();

    resetModuleEncoders();

    setIdleMode(true);

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));

      rotatePID.setP(.004);

      thetaPID.setP(0);

      xPID.setP(1.0);

      yPID.setP(0);

    }

    // m_Field2d = new Field2d();

    // SmartDashboard.putData(m_Field2d);

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  /*
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * 
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * 
   * @param rot Angular rate of the robot.
   * 
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        this.m_fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, get180Rotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

  }

  public void setClosedLoop(boolean on) {
    m_frontLeft.m_isOpenLoop = !on;
    m_frontRight.m_isOpenLoop = !on;
    m_backLeft.m_isOpenLoop = !on;
    m_backRight.m_isOpenLoop = !on;
    isOpenLoop = !on;
  }

  @Override
  public void periodic() {
    boolean tunePIDOn = false;

    if (tunePIDOn) {

      tuneXPIDGains();
      tuneYPIDGains();
      tuneThetaPIDGains();

    }

    if (trajectoryRunning && Math.abs(yTrajStart - getY()) > 2) {
      autonomousCommand.cancel();
      stopModules();
    }

    // Update the odometry in the periodic block

    SmartDashboard.putNumber("XPIDCmd", xPID.getSetpoint());
    SmartDashboard.putNumber("YPIDCmd", yPID.getSetpoint());
    SmartDashboard.putNumber("ThPIDCmd", thetaPID.getSetpoint());
    SmartDashboard.putNumber("XPIDPErr", xPID.getPositionError());
    SmartDashboard.putNumber("YPIDPErr", yPID.getPositionError());
    SmartDashboard.putNumber("ThPIDPErr", thetaPID.getPositionError());

    updateOdometry();

    if (DriverStation.isEnabled()) {

      desiredStates[0] = m_frontLeft.getDesiredState()[0];
      desiredStates[1] = m_frontLeft.getDesiredState()[1];
      desiredStates[2] = m_frontRight.getDesiredState()[0];
      desiredStates[3] = m_frontRight.getDesiredState()[1];
      desiredStates[4] = m_backLeft.getDesiredState()[0];
      desiredStates[5] = m_backLeft.getDesiredState()[1];
      desiredStates[6] = m_backRight.getDesiredState()[0];
      desiredStates[7] = m_backRight.getDesiredState()[1];

      measuredStates[0] = m_frontLeft.getMeasuredState()[0];
      measuredStates[1] = m_frontLeft.getMeasuredState()[1];
      measuredStates[2] = m_frontRight.getMeasuredState()[0];
      measuredStates[3] = m_frontRight.getMeasuredState()[1];
      measuredStates[4] = m_backLeft.getMeasuredState()[0];
      measuredStates[5] = m_backLeft.getMeasuredState()[1];
      measuredStates[6] = m_backRight.getMeasuredState()[0];
      measuredStates[7] = m_backRight.getMeasuredState()[1];

    }
    SmartDashboard.putNumberArray("swerve/desiredStates", desiredStates);
    SmartDashboard.putNumberArray("swerve/measuredStates", measuredStates);

    if (moduleFaultSeen == 0) {
      moduleFaultSeen = m_frontLeft.getFaults() + m_frontRight.getFaults() + m_backLeft.getFaults()
          + m_backRight.getFaults();
      if (moduleStickyFaultSeen == 0) {
        moduleStickyFaultSeen = m_frontLeft.getStickyFaults() + m_frontRight.getStickyFaults()
            + m_backLeft.getStickyFaults() + m_backRight.getStickyFaults();
      }
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void updateOdometry() {
    Rotation2d temp = m_gyro.getRotation2d();
    // if (RobotBase.isSimulation())
    //   temp = new Rotation2d(Units.degreesToRadians(m_simAngle.get()));

    if (true) {

      /** Updates the field relative position of the robot. */

      m_poseEstimator.update(
          temp, // m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });
    }
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    m_poseEstimator.resetPosition(getHeadingRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition() },
        pose);

  }

  public void setAngleAdjustment(double adjustment) {
    m_gyro.setAngleAdjustment(adjustment);
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((m_gyro.getAngle()), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Rotation2d get180Rotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees() - fieldOrientOffset);
  }

  public float getGyroPitch() {
    return m_gyro.getPitch();
  }

  public boolean checkCANOK() {

    return RobotBase.isSimulation()

        || m_frontLeft.checkCAN()
            && m_frontRight.checkCAN()
            && m_backLeft.checkCAN()
            && m_backLeft.checkCAN();
  }

  public void resetModuleEncoders() {
    m_frontLeft.resetAngleToAbsolute();
    m_frontRight.resetAngleToAbsolute();
    m_backLeft.resetAngleToAbsolute();
    m_backRight.resetAngleToAbsolute();
  }

  /** Zeroes the heading of the robot. */
  public void resetGyro() {
    m_gyro.reset();
    // m_gyro.setAngleAdjustment(0);

  }

  public double getX() {
    return getEstimatedPosition().getX();
  }

  public double getY() {
    return getEstimatedPosition().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public boolean isbraked() {
    return m_frontLeft.m_driveMotor.getIdleMode() == IdleMode.kBrake;
  }

  public void setIdleMode(boolean brake) {

    m_frontLeft.setDriveBrakeMode(brake);
    m_frontLeft.setTurnBrakeMode(brake);
    m_frontRight.setDriveBrakeMode(brake);
    m_frontRight.setTurnBrakeMode(brake);
    m_backLeft.setDriveBrakeMode(brake);
    m_backLeft.setTurnBrakeMode(brake);
    m_backRight.setDriveBrakeMode(brake);
    m_backRight.setTurnBrakeMode(brake);

  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public Command clearFaults() {
    moduleFaultSeen = 0;
    moduleStickyFaultSeen = 0;
    m_pdp.clearStickyFaults();
    return Commands.sequence(Commands.runOnce(() -> m_frontLeft.clearFaults()),
        Commands.runOnce(() -> m_frontRight.clearFaults()),
        Commands.runOnce(() -> m_backLeft.clearFaults()),
        Commands.runOnce(() -> m_backRight.clearFaults()));
  }

  public double[] r2dToArray(Pose2d pose) {
    double[] temp = { 0, 0, 0 };
    temp[0] = pose.getX();
    temp[1] = pose.getY();
    temp[2] = pose.getRotation().getDegrees();
    return temp;
  }

  @Override
  public void simulationPeriodic() {

    ChassisSpeeds chassisSpeedSim = m_kinematics.toChassisSpeeds(

        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });

    /**
     * Need to find the degree change in 20 ms from angular radians per second
     * 
     * So radsper20ms = radspersec/50
     * degrees per rad = 360/2PI=57.3
     * degrees per 20ms = radsper20ms * degrees per rad
     * conversion is 57.3/50=114.6/100=1.15
     */

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.15;
    SmartDashboard.putNumber("CHSSM", chassisSpeedSim.omegaRadiansPerSecond);
    temp += m_simAngle.get();
    m_simAngle.set(temp);
    SmartDashboard.putNumber("SIMANGLE", m_simAngle.get());
    Unmanaged.feedEnable(20);
  }

  public void jogTurnModule(SwerveModuleSM i, double speed) {
    i.turnMotorMove(speed);
  }

  public void positionTurnModule(SwerveModuleSM i, double angle) {
    i.positionTurn(angle);
  }

  // public void driveModule(SwerveModuleSM i, double speed) {
  // i.driveMotorMove(speed);
  // }

  public boolean getTurnInPosition(SwerveModuleSM i, double targetAngle) {
    return i.turnInPosition(targetAngle);
  }

  public PIDController getRotatePID() {
    return rotatePID;
  }

  public PIDController getXPID() {
    return xPID;
  }

  public PIDController getYPID() {
    return yPID;
  }

  public PIDController getThetaPID() {
    return thetaPID;
  }

  public void setIsRotating(boolean on) {
    isRotating = on;
  }

  public void tuneYPIDGains() {

    if (yPID.getP() != Pref.getPref("PPYkP"))
      yPID.setP(Pref.getPref("PPYkP"));

    if (yPID.getI() != Pref.getPref("PPYkI"))
      yPID.setI(Pref.getPref("PPYkI"));

    if (yPID.getD() != Pref.getPref("PPYkD"))
      yPID.setD(Pref.getPref("PPYkD"));

  }

  public void tuneXPIDGains() {

    if (xPID.getP() != Pref.getPref("PPXkP"))
      xPID.setP(Pref.getPref("PPXkP"));

    if (xPID.getI() != Pref.getPref("PPXkI"))
      xPID.setI(Pref.getPref("PPXkI"));

    if (xPID.getD() != Pref.getPref("PPXkD"))
      xPID.setD(Pref.getPref("PPXkD"));

  }

  public void tuneThetaPIDGains() {

    if (thetaPID.getP() != Pref.getPref("PPThetakP"))
      thetaPID.setP(Pref.getPref("PPThetakP"));

    if (thetaPID.getI() != Pref.getPref("PPThetakI"))
      thetaPID.setI(Pref.getPref("PPThetakI"));

    if (thetaPID.getP() != Pref.getPref("PPThetakD"))
      thetaPID.setD(Pref.getPref("PPThetakD"));

  }

  public void tuneRotatePIDGains() {

    if (rotatePID.getP() != Pref.getPref("PPRotatekP"))
      rotatePID.setP(Pref.getPref("PPThetakP"));

    if (rotatePID.getI() != Pref.getPref("PPRotatekI"))
      rotatePID.setI(Pref.getPref("PPThetakI"));

    if (rotatePID.getP() != Pref.getPref("PPRotatekD"))
      rotatePID.setD(Pref.getPref("PPThetakD"));

  }

  public double getAnglefromThrottle() {
    return 0;
  }

  public double getGyrocompedPitch() {
    return 0;
  }

}
