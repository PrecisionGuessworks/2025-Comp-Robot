package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class MoveupArm extends Command {

  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  
  private int m_TestOption = 0;
  private Timer m_ejectTimer = new Timer();

  public MoveupArm(int TestOption, ElevatorSubsystem elevator, ArmSubsystem arm) {
    m_TestOption = TestOption;
    m_elevator = elevator;
    m_arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_elevator.setHeight(Constants.Elevator.stowHeight);
    // m_intake.setAngle(Constants.Intake.intakeDeployAngle);
    if (m_TestOption == 1) {
      m_arm.setArmAngle(Constants.Arm.armIntakeAngle);
    } else if (m_TestOption == 2){
      m_arm.setWristAngle(Constants.Arm.wristTestAngle);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_elevator.setHeight(Constants.Elevator.L2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setWristAngle(Constants.Arm.wristStowAngle);
    
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_ejectTimer.get() > 0.35;
  // }
}
