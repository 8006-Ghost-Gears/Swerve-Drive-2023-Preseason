// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax arm;
  private final PIDController pidController = new PIDController(0.1, 0, 0);

  public ArmSub() {
    arm = new CANSparkMax(Constants.CAN.arm, MotorType.kBrushless);

    arm.restoreFactoryDefaults();
    arm.setInverted(true);
    arm.setOpenLoopRampRate(0.9); // changes how fast you get 100%
    arm.getEncoder().setPosition(0);
  }

  public void setPower(double armpower) {
    arm.set(armpower);
  }

  public void stopArm() {
    arm.set(0);
  }

  public void ArmOut() {

    final double kArmTolerance = 2.0;

    double targetPosition = 0;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        arm.getPIDController().setP(0.1);
        arm.getPIDController().setI(0);
        arm.getPIDController().setD(0);

        pidController.setTolerance(kArmTolerance);

        arm.getPIDController().setOutputRange(-0.2, 0.2);

        arm.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

  public void ArmStowed() {

    final double kArmTolerance = 2.0;

    double targetPosition = 0;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        arm.getPIDController().setP(0.1);
        arm.getPIDController().setI(0);
        arm.getPIDController().setD(0);

        pidController.setTolerance(kArmTolerance);

        arm.getPIDController().setOutputRange(-0.2, 0.2);

        arm.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm", arm.getEncoder().getPosition());
  }
}
