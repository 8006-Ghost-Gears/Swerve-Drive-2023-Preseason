// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveSubsystem driveSub = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static PS4Controller driver = new PS4Controller(Constants.driver);


  JoystickButton _xButton = new JoystickButton(driver, 1);

  /*
      (here to remember!!!)
      * buttonNumber 1 is Square on a PS4 Controller
      * buttonNumber 2 is X on a PS4 Controller
      * buttonNumber 3 is Circle on a PS4 Controller
      * buttonNumber 4 is Triangle on a PS4 Controller
      * buttonNumber 5 is L1 on a PS4 Controller
      * buttonNumber 6 is R1 on a PS4 Controller
      * buttonNumber 7 is L2 on a PS4 Controller
      * buttonNumber 8 is R2 on a PS4 Controller
      * buttonNumber 9 is SHARE on a PS4 Controller
      * buttonNumber 10 is OPTIONS on a PS4 Controller
      * buttonNumber 11 is L3 on a PS4 Controller
      * buttonNumber 12 is R3 on a PS4 Controller
      * buttonNumber 13 is the PlayStaion Button on a PS4 Controller
      * buttonNumber 14 is the Touchpad on a PS4 Controller
   */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveSub.setDefaultCommand(new DriveCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
