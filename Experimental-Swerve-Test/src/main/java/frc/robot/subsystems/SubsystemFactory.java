package frc.robot.subsystems;

import frc.robot.Constants;

public class SubsystemFactory {
        public SwerveDriveSubsystem CreateSwerveDriveSubsystem() {

                SwerveDriveSubsystem subsystem = new SwerveDriveSubsystem(
                        Constants.CAN_ID.PIGEON_ID,
                        new int[] {
                                Constants.CAN_ID.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                                Constants.CAN_ID.FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                                Constants.CAN_ID.FRONT_LEFT_MODULE_STEER_ENCODER_ID
                        },new int[] {
                                Constants.CAN_ID.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                                Constants.CAN_ID.FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                                Constants.CAN_ID.FRONT_RIGHT_MODULE_STEER_ENCODER_ID
                        },
                        new int[] {
                                Constants.CAN_ID.BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
                                Constants.CAN_ID.BACK_LEFT_MODULE_STEER_MOTOR_ID,
                                Constants.CAN_ID.BACK_LEFT_MODULE_STEER_ENCODER_ID
                        },
                        new int[] {
                                Constants.CAN_ID.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
                                Constants.CAN_ID.BACK_RIGHT_MODULE_STEER_MOTOR_ID,
                                Constants.CAN_ID.BACK_RIGHT_MODULE_STEER_ENCODER_ID
                        },

                        Constants.DRIVE.SWERVE_MODULE_SHUFFLEBOARD_TAB_NAME
                );
                subsystem.configure(Constants.DRIVE.GET_SWERVE_DRIVE_CONFIG());

                return subsystem;
        }

       /*  Example of another subsystem
       
       public IntakeRollerSubsystem CreateIntakeRollerSubsystem() {
                IntakeRollerSubsystem subsystem = new IntakeRollerSubsystem(Constants.CAN_ID.MASTER_INTAKE_MOTOR_ID,
                                Constants.CAN_ID.FOLLOWER_INTAKE_MOTOR_ID);
                subsystem.configure(Constants.INTAKE_ROLLER.GET_INTAKE_CONFIG());
                return subsystem;
        }

        */
    }