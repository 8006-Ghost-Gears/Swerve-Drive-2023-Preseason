// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveSub extends SubsystemBase {
  /** Creates a new driveSub. */
  CANSparkMax frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
  public driveSub() {

    frontLeftMotor = new CANSparkMax(Constants.frontLeftMotor,MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.backLeftMotor,MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.frontRightMotor,MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.backRightMotor,MotorType.kBrushless);


    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    //frontLeftMotor.setOpenLoopRampRate(.2);
    //frontRightMotor.setOpenLoopRampRate(.2);
    //backLeftMotor.setOpenLoopRampRate(.2);
    //backRightMotor.setOpenLoopRampRate(.2);

    frontLeftMotor.setSmartCurrentLimit(25);
    frontRightMotor.setSmartCurrentLimit(25);
    backLeftMotor.setSmartCurrentLimit(25);
    backRightMotor.setSmartCurrentLimit(25);

    frontLeftMotor.setInverted(false);
backLeftMotor.setInverted(false);
frontRightMotor.setInverted(true);
backRightMotor.setInverted(true);

  }




  public void setPower(double left,double right) {

    frontLeftMotor.set(left);
    backLeftMotor.set(left);
    frontRightMotor.set(right);
    backRightMotor.set(right);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}