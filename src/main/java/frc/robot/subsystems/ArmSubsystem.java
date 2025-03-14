// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Serial;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.ImmutableAngle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  //public final DigitalInput m_beamBreak = new DigitalInput(Constants.Arm.beamBreakPort);

  static private final QuixCANCoder m_armCoder = 
      new QuixCANCoder(Constants.Arm.armCoderID, Constants.Arm.armMotorRatio, SensorDirectionValue.Clockwise_Positive);
  
      static double ArmStartingAngle = Constants.Arm.armStartingAngle;
     // : Units.rotationsToRadians(m_armCoder.getAbsPosition()); Constants.isSim ? 
  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Arm.rollerMotorID,
          Constants.Arm.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.rollerMotorInvert)
              .setSupplyCurrentLimit(50.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(Constants.Arm.rollerVelocityPIDSlot, Constants.Arm.rollerPositionPIDConfig));

  private final QuixTalonFX m_armMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setMotionMagicConfig(
                  Constants.Arm.ArmConstraints.maxVelocity,
                  Constants.Arm.ArmConstraints.maxAcceleration,
                  Constants.Arm.ArmMaxJerk,
                  Constants.Arm.armExpo_kV,
                  Constants.Arm.armExpo_kA)

              .setPIDConfig(Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
              .setBootPositionOffset(ArmStartingAngle)
              .setReverseSoftLimit(Constants.Arm.armMinAngle)
              .setForwardSoftLimit(Constants.Arm.armMaxAngle)
            //  .setFeedbackConfig(FeedbackSensorSourceValue.FusedCANcoder, 15, 0.0,Constants.Arm.armMotorRatio,Constants.Arm.armSensorRatio)
              );

private final QuixTalonFX m_wristMotor =
  new QuixTalonFX(
      Constants.Arm.wristMotorID,
      Constants.Arm.wristMotorRatio,
      QuixTalonFX.makeDefaultConfig()
          .setInverted(Constants.Arm.wristMotorInvert)
          .setBrakeMode()
          .setSupplyCurrentLimit(40.0)
          .setStatorCurrentLimit(80.0)
          .setMotionMagicConfig(
              Constants.Arm.WristConstraints.maxVelocity,
              Constants.Arm.WristConstraints.maxAcceleration,
              Constants.Arm.WristMaxJerk,
                  Constants.Arm.wristExpo_kV,
                  Constants.Arm.wristExpo_kA)
          .setPIDConfig(Constants.Arm.wristPositionPIDSlot, Constants.Arm.wristPositionPIDConfig)
          .setBootPositionOffset(Constants.Arm.wristStartingAngle)
          .setReverseSoftLimit(Constants.Arm.wristMinAngle)
          .setForwardSoftLimit(Constants.Arm.wristMaxAngle));

  private double m_armTargetAngle = ArmStartingAngle;
  private double m_wristTargetAngle = Constants.Arm.wristStartingAngle;
  private double setm_armTargetAngle = ArmStartingAngle;
  private double setm_wristTargetAngle = Constants.Arm.wristStartingAngle;
  private boolean hasPiece = true;
  private Timer m_lastPieceTimer = new Timer();

  public ArmSubsystem(Link2d ArmArmViz, Link2d ArmWristViz, Link2d ArmRollerViz) {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_ArmArmViz = ArmArmViz;
    m_ArmRollerViz = ArmRollerViz;
    m_ArmWristViz = ArmWristViz;
  }

  // public boolean hasPiece() {
  //   return m_beamBreak.get();
  // }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
    }

    public double getArmAngle() { 
    return Units.radiansToDegrees(m_armMotor.getSensorPosition()) * Constants.Arm.armMotorRatio.inverseReduction() + Units.radiansToDegrees(ArmStartingAngle) ;
    //: Units.rotationsToRadians(m_armCoder.getAbsPosition());   Constants.isSim ? 
    }
    public double getWristAngle() {
    return Units.radiansToDegrees(m_wristMotor.getSensorPosition())* Constants.Arm.wristMotorRatio.inverseReduction() + Units.radiansToDegrees(Constants.Arm.wristStartingAngle);
  }
  public double getRollerCurrent() {
    return m_rollerMotor.getSupplyCurrent();
  }

  public double getArmCoder(){
    return Units.rotationsToDegrees(m_armCoder.getAbsPosition());
  }

  public void setArmAngle(double targetAngle) {
    setm_armTargetAngle = targetAngle;
  }
  public void setWristAngle(double targetAngle) {
    setm_wristTargetAngle = targetAngle;
  }

  public void setHasPiece(boolean thasPiece) {
    hasPiece = thasPiece;
  }
  public boolean getHasPiece() {
    return hasPiece;
  }


  public void setArmRollerCurrent(double StatorCurrentLimit, double SupplyCurrentLimit) {
    m_rollerMotor.setStatorCurrentLimit(StatorCurrentLimit,SupplyCurrentLimit);
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_armMotor.getSensorPosition()) <= tolerance;
  }

  public boolean isrollerStalled() {
    //return Math.abs(m_rollerMotor.getSensorVelocity()) < Constants.Arm.rollerStallVelocity;
    //return m_rollerMotor.getSupplyCurrent() > Constants.Arm.rollerStallCurrent;
  return false;
  }

  public void setRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_rollerMotor.setPercentOutput(0.0);
    } else {
      m_rollerMotor.setVelocitySetpoint(
          Constants.Arm.rollerVelocityPIDSlot,
          velocity,
          Constants.Arm.rollerFeedforward.calculate(velocity));
    }
  }

  // public void disabledInit() {
  //   m_armMotor.setBrakeMode(true);
  // }

  // public void disabledExit() {
  //   m_armMotor.setBrakeMode(false);
  // }

  //private double elevatorLocation = 0; 

  @Override
  public void periodic() {


    if (getArmAngle() <= 115 && getArmAngle() >= 92 && Units.radiansToDegrees(setm_armTargetAngle) < 92 && !RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(.75))) { // might need check 
       m_armTargetAngle = Constants.Arm.armStowIntakeAngle;
      //System.out.println("1");
    }else if ((((getArmAngle() > 92)||((RobotContainer.elevator.getHeight() >= Constants.Elevator.armStowHeight))) && setm_armTargetAngle <= Constants.Arm.armStowAngle)||
    (setm_armTargetAngle >= Constants.Arm.armL1Score && RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(.75)))){
      m_armTargetAngle = setm_armTargetAngle;
      
    } else if ((RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(0.75)) && Units.radiansToDegrees(setm_armTargetAngle) >= Constants.Arm.armStowAngle)
    || (getArmAngle() > 92 && setm_armTargetAngle <= Constants.Arm.armMaxAngle && Units.radiansToDegrees(setm_armTargetAngle) >= Constants.Arm.armStowIntakeAngle)){
      m_armTargetAngle = setm_armTargetAngle;
      //System.out.println("3");
    } else if (getArmAngle() < 92 && Units.radiansToDegrees(setm_armTargetAngle) > 91 ) { // might need check 
      m_armTargetAngle = Constants.Arm.armStowAngle;
      //System.out.println("4");
    }
    // if (RobotContainer.elevator.getHeight() > Constants.Elevator.armStowHeight && setm_armTargetAngle <= Constants.Arm.armStowAngle){
    //    m_armTargetAngle = setm_armTargetAngle;
    // } else {
    //   m_armTargetAngle = Constants.Arm.armStowAngle;
    // }
    
  
    if (Units.radiansToDegrees(setm_wristTargetAngle) < Units.degreesToRadians(85)&&getArmAngle() < 95){ //RobotContainer.elevator.getHeight() >= Constants.Elevator.wristStowHeight && 
      m_wristTargetAngle = setm_wristTargetAngle;
    } else if (Math.abs(getWristAngle() - Units.radiansToDegrees(Constants.Arm.wristStartingAngle)) <= 2 && setm_wristTargetAngle == Constants.Arm.wristStartingAngle) {
      m_wristTargetAngle = setm_wristTargetAngle;
    }else if (getArmAngle() >= 95){
      m_wristTargetAngle = Constants.Arm.wristIntakeAngle;
    }else if (getArmAngle() <= 70){
      m_wristTargetAngle = setm_wristTargetAngle;
    }else if (RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(1))){
     m_wristTargetAngle = setm_wristTargetAngle;
    }else{
     m_wristTargetAngle = Constants.Arm.wristStowAngle;
    }
    // if (RobotContainer.elevator.getHeight() > Constants.Elevator.wristStowHeight && setm_wristTargetAngle <= Constants.Arm.wristStowAngle){
    //   m_wristTargetAngle = setm_wristTargetAngle;
    // } else if (RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(1))){ 
    //   m_wristTargetAngle = setm_wristTargetAngle;
    // } else {
    //   m_wristTargetAngle = Constants.Arm.wristStowAngle;
    // }

       //m_armTargetAngle = setm_armTargetAngle;
       //m_wristTargetAngle = setm_wristTargetAngle;

    // if(hasPiece){
    //   m_armMotor.setMotionMagicPositionSetpoint(
    //     Constants.Arm.armCoralPositionPIDSlot, m_armTargetAngle);
    //   m_wristMotor.setMotionMagicPositionSetpoint(
    //     Constants.Arm.wristCoralPositionPIDSlot, m_wristTargetAngle);
    // } else {
      // m_armMotor.setMotionMagicPositionSetpoint(
      //   Constants.Arm.armPositionPIDSlot, m_armTargetAngle);
      m_armMotor.setMotionMagicPositionSetpointExpo(
          Constants.Arm.armPositionPIDSlot, m_armTargetAngle);

      // m_wristMotor.setMotionMagicPositionSetpoint(
      //   Constants.Arm.wristPositionPIDSlot, m_wristTargetAngle);
      m_wristMotor.setMotionMagicPositionSetpointExpo(
        Constants.Arm.wristPositionPIDSlot, m_wristTargetAngle);
   // }
  if(Constants.ExtraInfo){
    SmartDashboard.putNumber(
        "Arm: Current Angle (deg)", Units.radiansToDegrees(m_armMotor.getSensorPosition()));
        SmartDashboard.putNumber(
        "Arm: Current CANcoder Angle (deg)", getArmCoder());
    SmartDashboard.putNumber(
        "Arm: Real Current Angle (deg)", getArmAngle());
    SmartDashboard.putNumber(
        "Arm: Target Angle (deg)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReference()));
    SmartDashboard.putNumber(
        "Arm: Target set Angle (deg)",
        Units.radiansToDegrees(m_armTargetAngle));
    SmartDashboard.putNumber(
        "Arm: Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getSensorVelocity()));
    SmartDashboard.putNumber(
        "Arm: Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReferenceSlope()));
    SmartDashboard.putNumber(
        "Arm: Current Roller Velocity (rad per sec)", m_rollerMotor.getSensorVelocity());


      SmartDashboard.putNumber(
        "Wrist: Current Angle (deg)", Units.radiansToDegrees(m_wristMotor.getSensorPosition()));
        SmartDashboard.putNumber(
        "Wrist: Real Current Angle (deg)", getWristAngle());
      SmartDashboard.putNumber(
        "Wrist: Target Angle (deg)",
        Units.radiansToDegrees(m_wristMotor.getClosedLoopReference()));
      SmartDashboard.putNumber(
        "Wrist: Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_wristMotor.getSensorVelocity()));
      SmartDashboard.putNumber(
        "Wrist: Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_wristMotor.getClosedLoopReferenceSlope()));
      SmartDashboard.putNumber(
        "Wrist: Current Roller Velocity (rad per sec)", m_rollerMotor.getSensorVelocity());
        SmartDashboard.putNumber(
        "Wrist: Target set Angle (deg)",
        Units.radiansToDegrees(m_wristTargetAngle));
  }

  if(Constants.ExtraInfo){
    m_rollerMotor.logMotorState();
    m_wristMotor.logMotorState();
    m_armMotor.logMotorState();
    m_armCoder.logSensorState();
  }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Arm.armMotorRatio.reduction(),
          Constants.Arm.simArmMOI,
          Constants.Arm.simArmCGLength,
          Constants.Arm.armMinAngle,
          Constants.Arm.armMaxAngle,
          true, // Simulate gravity
          ArmStartingAngle);

  private static final SingleJointedArmSim m_wrstSim =
  new SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      Constants.Arm.wristMotorRatio.reduction(),
      Constants.Arm.wristArmMOI,
      Constants.Arm.wristCgOffset,
      Constants.Arm.wristMinAngle,
      Constants.Arm.wristMaxAngle,
      false, // Simulate gravity
      Constants.Arm.wristStartingAngle);

  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Arm.simRollerMOI,
              Constants.Arm.rollerMotorRatio.reduction()),
          m_simMotor);

          
  // Visualization
  private final Link2d m_ArmArmViz;
  private final Link2d m_ArmRollerViz;
  private final Link2d m_ArmWristViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - ArmStartingAngle,
        // m_armSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

    m_wrstSim.setInput(m_wristMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_wrstSim.update(TimedRobot.kDefaultPeriod);
    m_wristMotor.setSimSensorPositionAndVelocity(
      m_wrstSim.getAngleRads() - Constants.Arm.wristStartingAngle - ArmStartingAngle,
      // m_wrstSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
      0.0,
      TimedRobot.kDefaultPeriod,
      Constants.Arm.wristMotorRatio);
    

    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

    // Update arm viz.
    m_ArmArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmArmPivotX,
            0,
            Rotation2d.fromRadians(m_armSim.getAngleRads() + Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()))));

    m_ArmWristViz.setRelativeTransform(
      new Transform2d(
          Constants.Viz.ArmArmLength,
          0.0,
          Rotation2d.fromRadians(
            //m_ArmWristViz.getRelativeTransform().getRotation().getRadians()
                  + m_wrstSim.getAngleRads()+ Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()) - Units.degreesToRadians(90))));

    m_ArmRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmWristLength,
            0.0,
            Rotation2d.fromRadians(
                m_ArmRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_rollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}