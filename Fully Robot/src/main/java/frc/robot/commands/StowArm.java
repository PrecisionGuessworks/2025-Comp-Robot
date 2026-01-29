// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class StowArm extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;

  public StowArm(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem) {
    m_elevator = elevatorSubsystem;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setWristAngle(Constants.Arm.wristStowAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println(m_arm.getArmAngle());
  // }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   m_arm.setArmAngle(Constants.Arm.armStowAngle);
  //   m_arm.setWristAngle(Constants.Arm.wristStowAngle);
  //   m_elevator.setHeight(Constants.Elevator.stowHeight);
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
