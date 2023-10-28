// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class CAN_ID {
    /**
     * The canbus the swerve modules are on
     * 
     * Use "" or "rio" for rio can bus
     * 
     * Use name of Canivore device to use Canivore
     */

    //public static final String DRIVE_CANBUS = "CANivore";

    //public static final int PNEUMATICS_HUB_ID = 1;

    public static final int PIGEON_ID = 5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 11;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 12;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 19;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 14;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 20;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 15;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 16;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 21;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 17;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 18;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 22;
  }

  public static final class DRIVE {
    public static SwerveDriveSubsystem.Configuration GET_SWERVE_DRIVE_CONFIG() {
        SwerveDriveSubsystem.Configuration config = new SwerveDriveSubsystem.Configuration();

        config.m_trackwidthMeters = 0.50165;
        config.m_wheelbaseMeters = 0.55245;

        config.m_maxVoltage = 10.0;
        config.m_maxVelocityMetersPerSecond = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        config.m_robotCentricMaxVelocityPerSecond = config.m_maxVelocityMetersPerSecond / 2;


        config.m_maxAngularVelocityRadiansPerSecond = config.m_maxVelocityMetersPerSecond /
                Math.hypot(config.m_trackwidthMeters / 2.0, config.m_wheelbaseMeters / 2.0);
        config.m_robotCentricMaxAngularVelocityRadiansPerSecond = config.m_maxAngularVelocityRadiansPerSecond / 2;

        config.m_maxAngularAccelerationRadiansPerSecondSquared = config.m_maxAngularVelocityRadiansPerSecond / 3.0;

        config.m_xController = new PIDController(8.0, 0, 0); // .56122 2.2
        config.m_yController = new PIDController(8.0, 0, 0); // .56122
        config.m_thetaController = new PIDController(6.0, 0, 0); // 2.15

        config.m_sensorPositionCoefficient = 2.0 * Math.PI / Constants.DRIVE.TICKS_PER_ROTATION
                * SdsModuleConfigurations.MK4I_L2.getSteerReduction();

        /**
         * State measurement standard deviations. Left encoder, right encoder, gyro
         * Increase these numbers to trust them less
         */
        config.m_stateStdDevs = VecBuilder.fill(0.9, 0.9, 0.0);

        /**
         * Local measurement standard deviations. Vision X, Y, theta.
         * Increase these numbers to trust them less
         */
        config.m_visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, 1);

        config.m_visionToleranceMeters = 0.1524;

        config.m_autoAlignDriveController = new ProfiledPIDController(
            8.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2, 1));

        config.m_autoAlignThetaController = new ProfiledPIDController(
            6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2, 1));  
                
        return config;
    }

    // CONFIGURE THE FOLLOWING ON EACH CANCODER
        // absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
        // sensorDirection = false;
        // initializationStrategy = bootToAbsValue;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(8.79 + 180);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(314.38 - 180);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(306.3 - 180);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(163.04 + 180);

        public static final double BALANCE_LEVEL_DEGREES = 3.5;
        public static final double BALANCE_FULL_TILT_DEGREES = 15;
        public static final double BALANCE_KP = 0.015;
        public static final double BALANCE_MAX_POWER = 0.3;
        public static final double BALANCE_WAIT_MILLIS = 500;
        public static final double BALANCE_DENOMINATOR_MULTIPLIER = 7.5;

        public static final double TICKS_PER_ROTATION = 4096.0;

        public static final double WAIT_FOR_ZERO_TIME_MILLIS = 250;

        public static final double ENCODER_SYNC_ACCURACY_RADIANS = 0.05;

        public static final PIDController ROTATE_TO_TARGET_CONTROLLER = new PIDController(0.5, 0, 0.0001);

        public static final double ROTATE_MAXSPEED_RADIANS_PER_SECOND = 0.91;
        public static final double SYNC_ENCODER_LIMIT_MS = 10000;

        /*lic static final double DEAD_RECKONING_TRANSLATION_METERS_PER_SECOND = 1;
        public static final double DEAD_RECKONING_ROTATION_RADIANS_PER_SECOND = 0.1;

        public static final ChassisSpeeds DEAD_RECKONING_X_CHASSIS_SPEEDS = new ChassisSpeeds(
                DEAD_RECKONING_TRANSLATION_METERS_PER_SECOND, 0, 0);
        public static final ChassisSpeeds DEAD_RECKONING_Y_CHASSIS_SPEEDS = new ChassisSpeeds(0,
                DEAD_RECKONING_TRANSLATION_METERS_PER_SECOND, 0);
        public static final ChassisSpeeds DEAD_RECKONING_ROTATION_CHASSIS_SPEEDS = new ChassisSpeeds(0, 0,
                DEAD_RECKONING_ROTATION_RADIANS_PER_SECOND);

                */

        public static final String SWERVE_MODULE_SHUFFLEBOARD_TAB_NAME = "Drivetrain";

        public static final double AUTO_LINEUP_RANGE_METERS = 3;

        // Path constraints
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(2.5,
                1.5);
        public static final PathConstraints GRID_ZERO_PATH_CONSTRAINTS = new PathConstraints(2, 1);
   
        //public static final double TIME_TO_COAST_SECONDS = 10;
    }

  public static final class CONTROLLER {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DRIVE_CONTROLLER_DEADBAND = 0.05;
    //public static final double OPERATOR_CONTROLLER_DEADBAND = 0.1;

    //public static final double RUMBLE_INTENSITY = 0.5;
    //public static final double RUMBLE_TIMEOUT_SECONDS_ON_TELEOP_AUTO = 1;

    //public static final int BUTTON_BOARD_NUM_ROWS = 3;
    //public static final int BUTTON_BOARD_NUM_COLS = 9;
    //public static final double BUTTON_BOARD_JOYSTICK_MAX_VALUE = 32767;
}

/*public static final class COMPRESSOR {
  public static final int MIN_PRESSURE_PSI = 100;
  public static final int MAX_PRESSURE_PSI = 115;
}
*/

}
