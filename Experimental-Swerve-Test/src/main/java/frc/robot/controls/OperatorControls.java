package frc.robot.controls;

import frc.robot.Constants.CONTROLLER;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * These are the controls for the gunner.
 * 
 * @category Drive
 */
public class OperatorControls {

    private static OperatorControls s_instance;

    public static OperatorControls getInstance() {
        return s_instance;
    }

    PS4Controller m_controller;


    /**
     * @param builder The OperatorControlsBuilder object
     */
    public OperatorControls(PS4Controller controller) {
        s_instance = this;

        m_controller = controller;

        //JoystickButton Intake = new JoystickButton(controller, 5);

        mapControls();
    }

    public void mapControls() {
        
        //IntakeCommands

        //Intake.whileTrue(new IntakeCommand());
    }

}