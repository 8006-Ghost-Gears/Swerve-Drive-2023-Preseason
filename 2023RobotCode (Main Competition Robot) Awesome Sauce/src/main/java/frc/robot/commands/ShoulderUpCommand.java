// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShoulderUpCommand extends CommandBase {
  
  public ShoulderUpCommand() {
    addRequirements(RobotContainer.mShoulderSub);
  }
  
  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    RobotContainer.mShoulderSub.shoulderUp(0.2);
  }
  
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mShoulderSub.stopShoulder();
  }
}