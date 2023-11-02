// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drive. */
  CANSparkMax leftMotorBack, leftMotorFront, rightMotorBack, rightMotorFront;
  public DriveSubsystem() {
    leftMotorBack = new  CANSparkMax(Constants.leftMotorBack, MotorType.kBrushless);
    leftMotorFront = new  CANSparkMax(Constants.leftMotorFront, MotorType.kBrushless);
    rightMotorBack = new  CANSparkMax(Constants.rightMotorBack, MotorType.kBrushless);
    rightMotorFront = new  CANSparkMax(Constants.rightMotorFront, MotorType.kBrushless);

    leftMotorBack.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
