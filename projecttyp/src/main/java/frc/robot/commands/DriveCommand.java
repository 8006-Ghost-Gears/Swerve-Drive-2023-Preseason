// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase {
  /** Creates a new Drive. */
  public DriveCommand() {
    //addRequirements(system);
    addRequirements(RobotContainer.driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double[] getPower(double left, double right){
    final double[] power = {
      Math.signum(left) * Math.pow(left, 2),
      Math.signum(right) * Math.pow(right, 2)
    };
    return power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftSide = 0;
    double rightSide = 0;

    leftSide -= RobotContainer.driver.getLeftY();
    rightSide -= RobotContainer.driver.getRightY();

    double[] power = getPower(leftSide, rightSide);

    RobotContainer.driveSub.setPower(power[0], power[1]);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
