// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class AlgeaWack extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  private int pastscoreheight = 0;

  public AlgeaWack(
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
    //m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_arm.setArmAngle(Constants.Arm.armWackAngle);
    m_arm.setWristAngle(Constants.Arm.wristWackAngle);
    pastscoreheight = 0;
  }

  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d m_pose = RobotContainer.drivetrain.getState().Pose;
    Pose2d targetpose = new Pose2d(4.5,4,new Rotation2d(0));
    //    && m_arm.getWristAngle() < 91 && m_arm.getArmAngle() < 91
    // 4 >= PhotonUtils.getDistanceToPose(m_pose,targetpose)
    if (3 >= PhotonUtils.getDistanceToPose(m_pose,targetpose)||Constants.PoseSoring){
      //System.out.println("CoralMoveScore");
      if(pastscoreheight != m_elevator.getHeightLocation()){
        //System.out.println("part");
        if(m_elevator.getHeightLocation() == 1){
          pastscoreheight = 2;
          m_elevator.setHeight(Constants.Elevator.L2Algae);
        } else if(m_elevator.getHeightLocation() == 2){
          pastscoreheight = 2;
          m_elevator.setHeight(Constants.Elevator.L2Algae);
        } else if(m_elevator.getHeightLocation() == 3){
          pastscoreheight = 3;
          m_elevator.setHeight(Constants.Elevator.L3Algae);
        } else if(m_elevator.getHeightLocation() == 4){
          pastscoreheight = 3;
          m_elevator.setHeight(Constants.Elevator.L3Algae);
        }
       // m_elevator.setHeight(Constants.Elevator.L3Algae);
      }
    } else {
      m_elevator.setHeight(Constants.Elevator.stowHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmAngle(Constants.Arm.armWackAfterAngle);
    m_arm.setWristAngle(Constants.Arm.wristWackAfterAngle);
    //m_elevator.setHeight(Constants.Elevator.stowHeight);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
