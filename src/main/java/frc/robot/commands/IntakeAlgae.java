package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAlgae extends Command {
  private final IntakeSubsystem m_intake;
  private int m_intakemode = 0;

  public IntakeAlgae(IntakeSubsystem intakeSubsystem, int intakemode) {
    m_intake = intakeSubsystem;
    m_intakemode = intakemode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setRollerCurrnt(30);
    if ((m_intakemode == 0 && m_intake.hasPiece() == false)||m_intakemode == 1) { //intake
      m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
      m_intake.setAngle(Constants.Intake.intakeDeployAngle);
    } else{ //outtake
      m_intake.setAngle(Constants.Intake.intakeScoreAngle);
      m_intake.setRollerVelocity(Constants.Intake.outtakeRollerVelocity);
      m_intake.setHasPiece(false);
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerCurrnt(5);
    m_intake.setAngle(Constants.Intake.intakeStowAngle);
    if (m_intake.hasPiece()==true||m_intakemode == 1) { 
      m_intake.setRollerVelocity(Constants.Intake.holdRollerVelocity);     
    } else{ 
      m_intake.setRollerVelocity(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intake.getRollerCurrent() > 40) {
      m_intake.setHasPiece(true);
      return true;
    } else {
      m_intake.setHasPiece(false);
      return false;
    }
  }

}
