// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.*;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.SwerveDriveControls;
import frc.robot.shuffleboard.AutoCommandChooser;
import frc.robot.state.RobotState;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveDriveSubsystem m_drivetrainSubsystem;
  private final Compressor m_compressor;

  private AutoCommandChooser m_autoCommandChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotState.robotInit();

    // Create subsystems
    SubsystemFactory subsystemFactory = new SubsystemFactory();
    //subsystemFactory.CreateIntakeRollerSubsystem();

    m_drivetrainSubsystem = subsystemFactory.CreateSwerveDriveSubsystem();

    // Create operator controls and drive controls
    SwerveDriveControls driveControls = new SwerveDriveControls(
        new PS4Controller(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT), Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
    OperatorControls operatorControls = new OperatorControls(new PS4Controller(Constants.CONTROLLER.OPERATOR_CONTROLLER_PORT));

    // Set default commands
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        driveControls));

    /* // Set default pipeline
    DualLimelightManagerSubsystem.getInstance().setAprilTagPipelineActive();

    // Setup compressor
    m_compressor = new Compressor(Constants.CAN_ID.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH);
    m_compressor.enableAnalog(Constants.COMPRESSOR.MIN_PRESSURE_PSI,
        Constants.COMPRESSOR.MAX_PRESSURE_PSI);
    // m_compressor.disable();
    */

    m_autoCommandChooser = new AutoCommandChooser();

  }

 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}