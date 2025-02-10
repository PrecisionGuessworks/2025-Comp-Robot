// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.Moveup;
import frc.robot.commands.StowArm;
import frc.robot.Constants.Climber;
import frc.robot.commands.AlgeaWack;
import frc.robot.commands.ClimbSet;
import frc.robot.commands.CoralMoveScore;
import frc.robot.commands.CoralMoveStow;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.Drive.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
    private double MaxSpeed = MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    private double MaxSpeedUp = 0.6*MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed when arm is up
    private double MaxAngularRate = RotationsPerSecond.of(MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * SnapRotationDeadband) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors 
          //  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use motion magic control for steer motors

    private PowerDistribution powerDistribution = new PowerDistribution();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;


Map<String, Command> robotCommands  = new HashMap<String, Command>();



private static  final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(80.0), Units.inchesToMeters(120.0), 1.0);

private static  final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Elevator viz
  private  static final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private static  final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              6.0,
              Color.kLightGreen));
// Intake viz
private static final Link2d intakeArmViz =
robotViz.addLink(
    new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
private static final Link2d intakeRollerViz =
intakeArmViz.addLink(
    new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private static final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private static final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));

    
// Climber viz
private static final Link2d climberFrameViz =
robotViz.addLink(
    new Link2d(
        robotViz,
        "Climber Base",
        Constants.Viz.climberBaseLength,
        4.0,
        Color.kGreen,
        new Transform2d(
            Constants.Viz.climberBaseX,
            Constants.Viz.climberBaseY,
            Constants.Viz.climberAngle)));
private static final Link2d climberCarriageViz =
climberFrameViz.addLink(
    new Link2d(
        robotViz,
        "Climber Carriage",
        Constants.Viz.climberCarriageLength,
        6.0,
        Color.kLightGreen));


        public static final ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorCarriageViz);
        public static final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
        public static final ArmSubsystem arm = new ArmSubsystem(ArmArmViz,ArmWristViz,ArmWheelViz);
        public static final ClimberSubsystem climber = new ClimberSubsystem(climberCarriageViz);








    public RobotContainer() {

        robotCommands.put("IntakePiece", new IntakeAlgae(intake,1).withTimeout(2.5));
        robotCommands.put("CoralMoveScore", new CoralMoveScore(intake, elevator, arm).withTimeout(3));
        robotCommands.put("CoralMoveStow", new CoralMoveStow(intake, elevator, arm));
        robotCommands.put("IntakeCoral", new IntakeCoral(elevator, arm));
    
        NamedCommands.registerCommands(robotCommands);




        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto", autoChooser);
        angle.HeadingController.setPID( PRotation,  IRotation , DRotation);
        configureBindings();

    


        SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
        });
          SmartDashboard.putNumber("Time",Timer.getMatchTime());
          SmartDashboard.putNumber("Time2",DriverStation.getMatchTime());
          SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CAN",RobotController.getCANStatus().percentBusUtilization * 100.0);
          SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Power Distribution Panel", powerDistribution);

        PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();;
        
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        driver.leftBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(intake, elevator, arm), pathfindingCommand(true)));
        driver.rightBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(intake, elevator, arm), pathfindingCommand(false)));
        driver.leftBumper().onFalse(new CoralMoveStow(intake, elevator, arm));
        driver.rightBumper().onFalse(new CoralMoveStow(intake, elevator, arm));

        //driver.y().whileTrue(new ClimbSet(climber));
        //driver.x().whileTrue(pathfindingtofollowCommand());
        driver.rightTrigger().whileTrue(new IntakeCoral(elevator, arm));
        driver.leftTrigger().whileTrue(new IntakeAlgae(intake, 0));
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        

        // driver.y().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        // );
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(-90))))
        // );
        // driver.a().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(-180))))
        // );
        // driver.x().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(-270))))
        // );


        driver.a().whileTrue(new AlgeaWack(elevator, arm));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        operator.leftTrigger().whileTrue(new IntakeAlgae(intake, 1));
        operator.rightTrigger().whileTrue(new IntakeAlgae(intake, 2));
        operator.rightBumper().whileTrue(new StowArm(elevator, arm));
        operator.leftBumper().and(operator.a()).onTrue(new ClimbSet(1, climber, intake, elevator, arm));
        operator.leftBumper().and(operator.b()).onTrue(new ClimbSet(2, climber, intake, elevator, arm));
        operator.leftBumper().and(operator.x()).onTrue(new ClimbSet(3, climber, intake, elevator, arm));
        
        



        //drivetrain.registerTelemetry(logger::telemeterize);
    }










// --------------------------------------------------------- Commands --------------------------------------------------------- 




    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    public Rotation2d targetangle() {
        /* First put the drivetrain into auto run mode, then run the auto */
        SwerveDriveState state = drivetrain.getState();
        Pose2d pose = state.Pose;
        pose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
        Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        System.out.println(PhotonUtils.getYawToPose(pose,targetpose));
        return PhotonUtils.getYawToPose(pose,targetpose);
        
    }

    double X;
    double Y;
    double VX;
    double VY;
    double intercpet = Math.tan(Units.degreesToRadians(30))*4.5;
    double intercpetRed = Math.tan(Units.degreesToRadians(30))*13;
    double slope = Math.tan(Units.degreesToRadians(30));
    // Create the constraints to use while pathfinding
    
    private Command pathfindingCommand(boolean left) {

       PIDController xController = new PIDController(Constants.Pose.PTranslationSlow, Constants.Pose.ITranslationSlow, Constants.Pose.DTranslationSlow);
       PIDController yController = new PIDController(Constants.Pose.PTranslationSlow, Constants.Pose.ITranslationSlow, Constants.Pose.DTranslationSlow);

       Pose2d targetPose = Constants.Pose.Ablue; // Example target pose

       return new Command() {
           @Override
           public void initialize() {
               xController.reset();
               yController.reset();
           }

           @Override
           public void execute() {
            Pose2d currentPose = drivetrain.getState().Pose;
            ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;
            X = currentPose.getTranslation().getX();
            Y = currentPose.getTranslation().getY();
            VX = currentSpeeds.vxMetersPerSecond;
            VY = currentSpeeds.vyMetersPerSecond;
               double xOutput = 2 * xController.calculate(X, targetPose.getX());
               double yOutput = 2 * yController.calculate(Y, targetPose.getY());
            //    System.out.println(xOutput);
            //    System.out.println(yOutput);
               drivetrain.applyRequest(() -> 
                   angle.withVelocityX(xOutput)
                        .withVelocityY(yOutput)
                        .withTargetDirection(targetPose.getRotation())
               ).execute();
           }

           @Override
           public void end(boolean interrupted) {
               drivetrain.applyRequest(() -> 
                   angle.withVelocityX(0)
                        .withVelocityY(0)
                        .withTargetDirection(targetPose.getRotation())
               ).execute();
               xController.close();
                yController.close();
           }

        //    @Override
        //    public boolean isFinished() {
        //        return xController.atSetpoint() && yController.atSetpoint();
        //    }
       };
    }

    // private Command pathfindingCommand(boolean left) {
    //     // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //     Map<String, Command> commandMap = new HashMap<>();
    //     commandMap.put("error", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Error,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Ablue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Ablue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Bblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Bblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Cblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Cblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Dblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Dblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Eblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Eblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Fblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Fblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Gblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Gblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Hblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Hblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Iblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Iblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Jblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Jblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Kblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Kblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Lblue", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Lblue,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Ared", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Ared,
    //         Constants.Pose.constraints,
    //         0.0
    //     )); 
    //     commandMap.put("Bred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Bred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Cred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Cred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Dred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Dred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Ered", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Ered,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Fred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Fred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Gred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Gred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Hred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Hred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Ired", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Ired,
    //         Constants.Pose.constraints,
    //         0.0
    //     )); 
    //     commandMap.put("Jred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Jred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Kred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Kred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));
    //     commandMap.put("Lred", AutoBuilder.pathfindToPose(
    //         Constants.Pose.Lred,
    //         Constants.Pose.constraints,
    //         0.0
    //     ));

        
        

    //     return Commands.select(commandMap, () -> {
    //         Pose2d currentPose = drivetrain.getState().Pose;
    //         ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;
    //         X = currentPose.getTranslation().getX() + currentSpeeds.vxMetersPerSecond * Constants.Pose.XvelocityFactor;
    //         Y = currentPose.getTranslation().getY() + currentSpeeds.vyMetersPerSecond * Constants.Pose.YvelocityFactor;
    //         System.out.println(X);
    //         System.out.println(Y);
            
    //         if(DriverStation.getAlliance().get() == Alliance.Blue){ 

    //             if (Y <= -X*slope + intercpet+4 && Y >= X*slope - intercpet+4 && left && X <= 4.5) {
    //             System.out.println("Ablue");
    //             return "Ablue";
    //             } else if (Y <= -X*slope + intercpet+4 && Y >= X*slope - intercpet+4 && !left && X <= 4.5) {
    //             System.out.println("Bblue");
    //             return "Bblue";
    //             } else if (Y <= X*slope - intercpet+4 && X <= 4.5 && left) {
    //             System.out.println("Cblue");
    //             return "Cblue";
    //             } else if (Y <= X*slope - intercpet+4 && X <= 4.5 && !left) {
    //             System.out.println("Dblue");
    //             return "Dblue";
    //             } else if (Y <= -X*slope + intercpet+4 && X >= 4.5 && left) {
    //             System.out.println("Eblue");
    //             return "Eblue";
    //             } else if (Y <= -X*slope + intercpet+4 && X >= 4.5 && !left) {
    //             System.out.println("Fblue");
    //             return "Fblue";
    //             } else if (Y >= -X*slope + intercpet+4 && Y <= X*slope - intercpet+4 && left && X >= 4.5) {
    //             System.out.println("Gblue");
    //             return "Gblue";
    //             } else if (Y >= -X*slope + intercpet+4 && Y <= X*slope - intercpet+4 && !left && X >= 4.5) {
    //             System.out.println("Hblue");
    //             return "Hblue";
    //             } else if (Y >= X*slope - intercpet+4 && X >= 4.5 && left) {
    //             System.out.println("Iblue");
    //             return "Iblue";
    //             } else if (Y >= X*slope - intercpet+4 &&  X >= 4.5 && !left) {
    //             System.out.println("Jblue");
    //             return "Jblue";
    //             } else if (Y >= -X*slope + intercpet+4 && X <= 4.5 && left) {
    //             System.out.println("Kblue");
    //             return "Kblue";
    //             } else if (Y >= -X*slope + intercpet+4 && X <= 4.5 && !left) {
    //             System.out.println("Lblue");
    //             return "Lblue";
    //             } else {
    //             System.out.println("error");
    //             return "error";
    //             }

    //         } else {
    //         if (Y <= -X*slope + intercpetRed+4 && Y >= X*slope - intercpetRed+4 && left && X <= 13.0) {
    //                 System.out.println("Ared");
    //                 return "Ared";
    //                 } else if (Y <= -X*slope + intercpetRed+4 && Y >= X*slope - intercpetRed+4 && !left && X <= 13.0) {
    //                 System.out.println("Bred");
    //                 return "Bred";
    //                 } else if (Y <= X*slope - intercpetRed+4 && X <= 13.0 && left) {
    //                 System.out.println("Cred");
    //                 return "Cred";
    //                 } else if (Y <= X*slope - intercpetRed+4 && X <= 13.0 && !left) {
    //                 System.out.println("Dred");
    //                 return "Dred";
    //                 } else if (Y <= -X*slope + intercpetRed+4 && X >= 13.0 && left) {
    //                 System.out.println("Ered");
    //                 return "Ered";
    //                 } else if (Y <= -X*slope + intercpetRed+4 && X >= 13.0 && !left) {
    //                 System.out.println("Fred");
    //                 return "Fred";
    //                 } else if (Y >= -X*slope + intercpetRed+4 && Y <= X*slope - intercpetRed+4 && left && X >= 13.0) {
    //                 System.out.println("Gred");
    //                 return "Gred";
    //                 } else if (Y >= -X*slope + intercpetRed+4 && Y <= X*slope - intercpetRed+4 && !left && X >= 13.0) {
    //                 System.out.println("Hred");
    //                 return "Hred";
    //                 } else if (Y >= X*slope - intercpetRed+4 && X >= 13.0 && left) {
    //                 System.out.println("Ired");
    //                 return "Ired";
    //                 } else if (Y >= X*slope - intercpetRed+4 &&  X >= 13.0 && !left) {
    //                 System.out.println("Jred");
    //                 return "Jred";
    //                 } else if (Y >= -X*slope + intercpetRed+4 && X <= 13.0 && left) {
    //                 System.out.println("Kred");
    //                 return "Kred";
    //                 } else if (Y >= -X*slope + intercpetRed+4 && X <= 13.0 && !left) {
    //                 System.out.println("Lred");
    //                 return "Lred";
    //                 } else {
    //                 System.out.println("error");
    //                 return "error";
    //                 }
                
    //             }
            


    //     });
    // }

        
     private Command pathfindingtofollowCommand() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("Testpath");
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
                 
        );
    }
}
