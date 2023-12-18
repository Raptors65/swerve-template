// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.simulation.SimConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CanConstants {

    // TODO: device IDs
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_CANCODER = 6;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 231.24; // TODO: this should be equal to the value of
                                                                        // the CANCoder when the wheel is facing
                                                                        // forward.

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_CANCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 317.725; // TODO: this should be equal to the value of
                                                                          // the CANCoder when the wheel is facing
                                                                          // forward.

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_CANCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 182.28; // TODO: this should be equal to the value of the
                                                                       // CANCoder when the wheel is facing forward.

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CANCODER = 15;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 254.18; // TODO: this should be equal to the value of
                                                                        // the CANCoder when the wheel is facing
                                                                        // forward.

  }

  public class IDConstants {

    public static final int FRONT_LEFT_LOCATION = 0;
    public static final int FRONT_RIGHT_LOCATION = 1;
    public static final int REAR_LEFT_LOCATION = 2;
    public static final int REAR_RIGHT_LOCATION = 3;

  }

  public static final class DriveConstants {

    // TODO: reverse motors if necessary
    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    // TODO: update dimensions
    public static final double kTrackWidth = Units.inchesToMeters(22);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);

    private final static Translation2d m_frontLeftLocation = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_frontRightLocation = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    private final static Translation2d m_backLeftLocation = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_backRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final Translation2d[] kModuleTranslations = {
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    };

    public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public static final double moduleRadius = .5;

    public static final double moduleCircumference = Math.PI * moduleRadius * 2; // 3.1 approx

    public static final double kMaxSpeedMetersPerSecond = 3.25;

    public static final double timePerRevMaxSpeed = moduleCircumference / kMaxSpeedMetersPerSecond;// .9

    public static final double kMaxRotationRadiansPerSecond = 2 * Math.PI * timePerRevMaxSpeed;

    public static final double kMaxRotationRadiansPerSecondSquared = kMaxRotationRadiansPerSecond;

    public static double kTurnP = .016;
    public static double kTurnI = 0;
    public static double kTurnD = 0;

  }

  public static class SwerveTransformPID {
    public static final double PID_XKP = 2;
    public static final double PID_XKI = 0.0;
    public static final double PID_XKD = 0.0;
    public static final double PID_YKP = 2;
    public static final double PID_YKI = 0.0;
    public static final double PID_YKD = 0.0;
    public static final double PID_TKP = 9.0;
    public static final double PID_TKI = 0.0;
    public static final double PID_TKD = 0.0;

    public static final double MAX_ANGULAR_VELOCITY = 1.0;
    public static final double MAX_ANGULAR_ACCELERATION = 1;
    public static final double STD_DEV_MOD = 2.0;
  }

  public static final class DriverConstants {

    public static double kTranslationSlew = 50.0;
    public static double kRotationSlew = 1.0;
    public static double kControllerDeadband = .025;
    public static double kControllerRotDeadband = .1;

  }

  public static final class ModuleConstants {

    // ModuleConfiguration MK4I_L1

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14 .122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;// 0.039198257811106

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;// 25.511337897182322

    public static final double kTurningDegreesPerEncRev =

        360 / mk4iL1TurnGearRatio;

    public static double kVoltCompensation = 12.6;

  }

  public static final class CurrentLimitConstants {

    public static final int turnMotorSmartLimit = 20;

    public static final int driveMotorSmartLimit = 20;

  }

  public final static class ModuleTuneConstants {

    public static final double kPModuleDriveController = 1.3e-6;
    public static final double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0;

    public static final double kPModuleTurningController = .004;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;

  }

  public static final class SYSIDConstants {
    // from Beta test
    public static final double ksDriveVoltSecondsPerMeter = .172;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 3.16;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.37;
    // sysid on module?
    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final int kArmControllerPort = 3;
    public static final int kTestControllerPort = 4;

  }

  public static final class PPConstants {
    // public static final double kMaxSpeedMetersPerSecond =
    // DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    // public static final double kMaxAngularSpeedRadiansPerSecond =
    // DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
    // / 10;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kPXController = 2;// 3.2 returned to manitowoc values
    public static final double kDXController = 0;
    public static final double kIXController = 0;

    public static final double kPYController = 2;
    public static final double kDYController = 0;
    public static final double kIYController = 0;

    public static final double kPThetaController = 1.25;// .35 returned to Manitowoc values
    public static final double kDThetaController = 0;
    public static final double kIThetaController = 0;

    public static final double kPRotateController = 0.025;
    public static final double kDRotateController = .0;
    public static final double kIRotateController = 0;

    public static final double kPStrafeController = 0.8;
    public static final double kDStrafeController = .0;
    public static final double kIStrafeController = 0;

  }

  public static final class drToTgtConstants {

    public static final double strkP = 1;
    public static final double strkD = 0;
    public static final double strkI = 0;

    public static final double rotkP = 1;
    public static final double rotkD = 0;
    public static final double rotkI = 0;

  }

  public static final class LEDConstants {

    public static final int LED_CONTROLLER_PORT = 1;
  }

  public static double redBlueYShift = Units.inchesToMeters(100);

  public static final Transform2d redToBlueTransform = new Transform2d(new Translation2d(0, redBlueYShift),
      new Rotation2d());

  public static class LoadStationPickupConstants {

    static Pose2d aprilTag5 = SimConstants.Tags.aprilTags[3].toPose2d();

    static Transform2d rightPickupT2d = new Transform2d(new Translation2d(0, .8), new Rotation2d());

    static Transform2d leftPickupT2d = new Transform2d(new Translation2d(0, -.8), new Rotation2d());

    public static Pose2d blueLeftTarget = aprilTag5.plus(rightPickupT2d);

    public static Pose2d blueRightTarget = aprilTag5.plus(leftPickupT2d);

    static Pose2d aprilTag6 = SimConstants.Tags.aprilTags[4].toPose2d();

    public static Pose2d redLeftTarget = aprilTag6.plus(rightPickupT2d);

    public static Pose2d redRightTarget = aprilTag6.plus(leftPickupT2d);

  }
}