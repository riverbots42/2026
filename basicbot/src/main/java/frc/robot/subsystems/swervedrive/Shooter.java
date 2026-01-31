package frc.robot.subsystems.swervedrive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkMax shoot = new SparkMax(40, MotorType.kBrushless);
    private final RelativeEncoder encoder1;
    private SparkClosedLoopController controller;
    private final SparkMaxConfig config;
    private Vision vision;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    public Shooter(PhotonCamera cam1)
    {
        encoder1 = shoot.getEncoder();
        controller = shoot.getClosedLoopController();
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        camera = cam1;
        //config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.002064500004053116,0.0,0.0);
        //config.encoder.positionConversionFactor(0.);
        //shoot.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        setDefaultCommand(
             runOnce(
                     () -> {
                       controller.setSetpoint(0, SparkBase.ControlType.kVelocity);
                     })
                 .andThen(run(() -> {}))
                 .withName("Idle"));

    }
    public Command set()
    {
        return run(() -> {
            double area = getArea();
            double distance = getDistance(area);
            double velocity = getVelocity(distance);
            System.out.println("Area:" + area);
            System.out.println("Distance: " + distance);
            System.out.println("Velocity:" + velocity);

            if(velocity != 0)
            {
                controller.setSetpoint(velocity, SparkBase.ControlType.kVelocity);
            }
            
        });
    }
    private double getArea()
    {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        //System.out.println("Size of List: " + result.size());
        if(result.size() != 0)
        {
            latestResult = result.get(0);
        }
        System.out.println(latestResult.getBestTarget().getArea());
        return latestResult.getBestTarget().getArea();
    }
    public double getVelocity(double distance)
    {
        if(distance == 0)
        {
            return 0;
        }
        return (-0.511905 * distance -106.52381);
    }
    public double getDistance(double area)
    {
        if(area == 0)
        {
            return 0.0;
        }
        return (65.29824* (Math.pow(area,-0.534256)));
    }


}
