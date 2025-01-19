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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Moveup;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.Drive.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
    private double MaxSpeed = MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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



private final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(60.0), Units.inchesToMeters(60.0), 1.0);

private final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Elevator viz
  private final Link2d elevatorFrameViz =
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
  private final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              5,
              Color.kLightGreen));
// Intake viz
private final Link2d intakeArmViz =
robotViz.addLink(
    new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
private final Link2d intakeRollerViz =
intakeArmViz.addLink(
    new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));

    

private final ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorCarriageViz);
private final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
private final ArmSubsystem arm =
      new ArmSubsystem(
          ArmArmViz,
          ArmWristViz,
          ArmWheelViz
          );








    public RobotContainer() {

        robotCommands.put("IntakePiece", new IntakePiece(intake, elevator).withTimeout(2.5));
    
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


    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(targetangle()))
        );

        joystick.y().whileTrue(pathfindingCommand());
        joystick.x().whileTrue(pathfindingtofollowCommand());
        joystick.b().whileTrue(new IntakeCoral(intake, elevator, arm));
        joystick.rightTrigger().whileTrue(new IntakePiece(intake, elevator));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(270))))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withTargetDirection(new Rotation2d(Math.toRadians(180))))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            angle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.toRadians(90))))
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

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

    private Command pathfindingCommand() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(3.764, 2.954  , Rotation2d.fromDegrees(60));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
    }


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
