package frc.robot.controls;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SwerveDriveControls {
    private static SwerveDriveControls s_instance;

    public static SwerveDriveControls getInstance() {
        return s_instance;
    }

    private PS4Controller m_controller;
    private double m_deadband;

    /*
 * Tips For Programmers
 * POV is the D-Pad, Example: POVButton handOpen = new POVButton(operator, 90);
 * The POV buttons are referred to by the angle. Up is 0, right is 90, down is 180, and left is 270.
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
 * 
 * https://www.chiefdelphi.com/t/make-motor-move-at-a-specific-rpm/396774
 * Click this link if your trying to send a motor to a certain position
 * 
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
 * Click this link if your trying to send a motor to a certain position
 * 
 * If you ever run into that problem like The Java Server Crashed 5 Times and will not be restarted, (worked for me)
 * then just delete all the WPILIB VS Codes and The Visual Studio Code that is on the pc, this is because Wpilib uses their own
 * VS Code when you install the latest version!
 */

    public SwerveDriveControls(PS4Controller controller, double deadband) {
        s_instance = this;

        m_controller = controller;
        m_deadband = deadband;

        JoystickButton Stabalize = new JoystickButton(controller, 5);
        

        mapControls();
    }

    public void mapControls() {
        // Zero swerve drive
       // m_backButton.whileTrue(new InstantCommand(() -> SwerveDriveSubsystem.getInstance().zeroGyroscope()));
       // m_startButton.whileTrue(new InstantCommand(() -> SwerveDriveSubsystem.getInstance().setGyroScope(180)));

        // Intake commands

        //
        Stabalize.whileTrue(new AutoBalanceCommand());
    }

    public double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public double modifyAxis(double value) {
        value = deadband(value, m_deadband);
        value = Math.copySign(value * value, value);
        return value;
    }

    
}