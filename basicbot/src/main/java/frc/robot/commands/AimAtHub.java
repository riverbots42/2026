package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;




public class AimAtHub extends Command {

    private PIDController rotController;    
    private SwerveSubsystem drivebase;
    private final static Pose2d redAllianceHub = new Pose2d(12.2, 4.0, new Rotation2d());
    private final static Pose2d blueAllianceHub = new Pose2d(4.625, 4.04, new Rotation2d());

    public AimAtHub(SwerveSubsystem drivebase) {
        rotController = new PIDController(0.04, 0, 0);
        this.drivebase = drivebase;
        addRequirements(drivebase);
        System.out.println("Contructor:)");
    }

    @Override
    public void initialize() {
        double yaw = PhotonUtils.getYawToPose(drivebase.getPose(), getAllianceHub()).getRadians();
        rotController.setSetpoint(yaw + drivebase.getHeading().getRadians());
        rotController.setTolerance(Math.toRadians(1));
        System.out.println("Init:)");
    }

    @Override
    public void execute() {
        double rotVal = rotController.calculate(drivebase.getHeading().getRadians()) * 100;
        drivebase.drive(new Translation2d(0, 0), -rotVal, false);
        System.out.println("Execute: " + rotVal);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(), 0, false);
        System.out.println("End:)");
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return rotController.atSetpoint();
    }

    public static Pose2d getAllianceHub() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                System.out.println("Red");
                return redAllianceHub;
            }
            if (ally.get() == Alliance.Blue) {
                System.out.println("Blue");
                return blueAllianceHub;
            }
        }
        return new Pose2d();
    }
}


