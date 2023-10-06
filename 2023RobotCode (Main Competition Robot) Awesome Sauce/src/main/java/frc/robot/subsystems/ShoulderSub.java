// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShoulderSub extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax shoulderLeft;
 CANSparkMax shoulderRight;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public ShoulderSub() {
    shoulderLeft = new CANSparkMax(Constants.CAN.shoulderLeft,MotorType.kBrushless);
    shoulderRight = new CANSparkMax(Constants.CAN.shoulderRight,MotorType.kBrushless);
    
    shoulderLeft.restoreFactoryDefaults();
    shoulderLeft.setInverted(false);
    shoulderLeft.getEncoder().setPosition(0);

  }



  public void shoulderDown(double speed) {
    if (shoulderLeft.getEncoder().getPosition() > -0.05) {  // Check if current position is greater than -1
      shoulderLeft.set(-speed);
      shoulderRight.set(-speed);
    } else {
      stopShoulder();
    }
  }
  public void shoulderUp(double speed) {
    if (shoulderLeft.getEncoder().getPosition() < 90) {  // Check if current position is less than 90
      shoulderLeft.set(speed);
      shoulderRight.set(speed);
    } else {
      stopShoulder();
    }
  }

  public void ShoudlerMid() {
    
    final double kShoulderTolerance = 2.0;

    double targetPosition = 0;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        shoulderLeft.getPIDController().setP(0.1);
        shoulderLeft.getPIDController().setI(0);
        shoulderLeft.getPIDController().setD(0);
        
        shoulderRight.getPIDController().setP(0.1);
        shoulderRight.getPIDController().setI(0);
        shoulderRight.getPIDController().setD(0);

        pidController.setTolerance(kShoulderTolerance);

        shoulderLeft.getPIDController().setOutputRange(-0.2, 0.2);
        shoulderRight.getPIDController().setOutputRange(-0.2, 0.2);

        shoulderLeft.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
        shoulderRight.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }

  public void ShoulderStowed() {
    
    final double kShoulderTolerance = 2.0;

    double targetPosition = 0;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        shoulderLeft.getPIDController().setP(0.1);
        shoulderLeft.getPIDController().setI(0);
        shoulderLeft.getPIDController().setD(0);
        
        shoulderRight.getPIDController().setP(0.1);
        shoulderRight.getPIDController().setI(0);
        shoulderRight.getPIDController().setD(0);

        pidController.setTolerance(kShoulderTolerance);

        shoulderLeft.getPIDController().setOutputRange(-0.2, 0.2);
        shoulderRight.getPIDController().setOutputRange(-0.2, 0.2);

        shoulderLeft.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
        shoulderRight.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
  

public void stopShoulder(){
  shoulderLeft.set(0);
  shoulderRight.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShoulderLeft", shoulderLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("ShoulderRight", shoulderRight.getEncoder().getPosition());

  }
}

