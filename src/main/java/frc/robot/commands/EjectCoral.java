// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class EjectCoral extends Command {
  private final ArmSubsystem m_arm;
  private boolean m_intake = true;

  public EjectCoral(Boolean intake,
      ArmSubsystem armSubsystem) {
    m_arm = armSubsystem;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmRollerCurrent(30, 30);
    
    if (m_intake){
      m_arm.setRollerVelocity(-100);
    } else {
      m_arm.setRollerVelocity(100);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {


  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setRollerVelocity(0);
    
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return trueendtrigger && m_elevator.isAtScore();
  // }
}