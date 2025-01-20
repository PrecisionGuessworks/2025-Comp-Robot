// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;


public class ScoreHeight extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  public int m_targetHeight;

  public ScoreHeight(
      int targetHeight,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem) {
    m_targetHeight = targetHeight;
    m_elevator = elevatorSubsystem;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_elevator.setHeightLocation(m_targetHeight);
   //m_arm.setArmAngle(Constants.Arm.armIntakeAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
}
