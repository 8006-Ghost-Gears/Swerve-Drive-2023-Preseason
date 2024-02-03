// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveSub extends SubsystemBase {
  /** Creates a new driveSub. */
  CANSparkMax frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-phantom");
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

  public void limelightTurn(double left, double right)
  {
    float Kp = -0.01f;  // Proportional control constant
    float min_command = 0.001f;

    double targetx = limelight.getEntry("tx").getDouble(0);

    double heading_error = -targetx;
    double steering_adjust = 0.0;

    if (Math.abs(heading_error) > 1.0) 
    {
        if (heading_error < 0) 
        {
            steering_adjust = Kp*heading_error + min_command;
        } 
        else 
        {
            steering_adjust = Kp*heading_error - min_command;
        }
    } 
    
    left += steering_adjust;
    right -= steering_adjust;

    setPower(left, right);

  }

  public double Estimate_Distance()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 45.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 9.0; 

    // distance from the target to the floor
    double goalHeightInches = 36.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  public void limelightGo(double left, double right)
  {
    float KpDistance = -0.01f;  // Proportional control constant for distance
    double current_distance = Estimate_Distance();  // see the 'Case Study: Estimating Distance' 

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-phantom");
    float distance_error = limelight.getEntry("ty").getFloat(0);

    float desired_distance = 35.0f;
    
    double distance_adjust = KpDistance * distance_error;

    left += distance_adjust;
    right += distance_adjust;

    setPower(left, right);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}