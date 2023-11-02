// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drive. */
  CANSparkMax leftMotorBack, leftMotorFront, rightMotorBack, rightMotorFront;
  public DriveSubsystem() {
    
    // [  Right Motors  ]  
    rightMotorBack = new CANSparkMax(Constants.rightMotorBack, MotorType.kBrushless);
    rightMotorFront = new CANSparkMax(Constants.rightMotorFront, MotorType.kBrushless);
    
    // [  Left Motors   ]
    leftMotorBack = new CANSparkMax(Constants.leftMotorBack, MotorType.kBrushless);
    leftMotorFront = new CANSparkMax(Constants.leftMotorFront, MotorType.kBrushless);
    
    // [  Restore Right Defaults  ]
    rightMotorBack.restoreFactoryDefaults();
    rightMotorFront.restoreFactoryDefaults();
    
    // [  Restore Left Defaults  ]
    leftMotorBack.restoreFactoryDefaults();
    leftMotorFront.restoreFactoryDefaults();

    // [  Limit  ]
    leftMotorFront.setSmartCurrentLimit(25);
    leftMotorBack.setSmartCurrentLimit(25);

    rightMotorFront.setSmartCurrentLimit(25);
    rightMotorBack.setSmartCurrentLimit(25);

    // []
    leftMotorBack.setInverted(false);
    leftMotorFront.setInverted(false);

    rightMotorBack.setInverted(true);
    rightMotorFront.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
