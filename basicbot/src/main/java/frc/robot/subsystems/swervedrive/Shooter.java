package frc.robot.subsystems.swervedrive;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class Shooter extends SubsystemBase
{
    SparkMax spinnerMax1 = new SparkMax(40, MotorType.kBrushless);
    SparkMax spinnerMax2 = new SparkMax(41, MotorType.kBrushless);
    SparkMax feederMax = new SparkMax(39, MotorType.kBrushless);
    SparkMax indexMax = new SparkMax(44, MotorType.kBrushless);

    private final SwerveSubsystem drivebase;

    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final RelativeEncoder indexEncoder;
    private final RelativeEncoder feederEncoder;

    private final SparkClosedLoopController controller1;
    private final SparkClosedLoopController controller2;
    private final SparkClosedLoopController controller3;
    private final SparkClosedLoopController indexController;
    private int indexDirectionalityCounter = 0;
    private final int indexDirectionalityMin = -20;
    private final int indexDirectionalityMax = 100;
    private double feedSpeed = .6;
    private final SparkMaxConfig config;
    private Vision vision;
    private PhotonPipelineResult latestResult;
    
    private InterpolatingDoubleTreeMap velocityMap;

    public Shooter(SwerveSubsystem drivebase)
    {
        encoder1 = spinnerMax1.getEncoder();
        encoder2 = spinnerMax2.getEncoder();
        feederEncoder = feederMax.getEncoder();
        indexEncoder = indexMax.getEncoder();

        indexController = indexMax.getClosedLoopController();
        controller1 = spinnerMax1.getClosedLoopController();
        controller2 = spinnerMax2.getClosedLoopController();
        controller3 = feederMax.getClosedLoopController();
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        this.drivebase = drivebase;
        velocityMap = new InterpolatingDoubleTreeMap();
        setupTreeMap();
        //config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.002064500004053116,0.0,0.0);
        //config.encoder.positionConversionFactor(0.);
        //shoot.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        setDefaultCommand(
             runOnce(
                     () -> {
                        controller1.setSetpoint(0, SparkBase.ControlType.kVelocity);
                        controller2.setSetpoint(0, SparkBase.ControlType.kVelocity);
                        controller3.setSetpoint(0, SparkBase.ControlType.kVelocity);
                        indexController.setSetpoint(0, SparkBase.ControlType.kVelocity);
                     })
                 .andThen(run(() -> {}))
                 .withName("Idle"));

    }
    private void setupTreeMap()
    {
        velocityMap.put(1.45,107.0);
        velocityMap.put(2.57,120.0);
        velocityMap.put(3.6,150.0);
        velocityMap.put(2.6,125.0);
        velocityMap.put(2.03,120.0);
        velocityMap.put(2.11,118.0);

    }
    double velocity = 50;
    public void incrementVelocity() { 
        velocity = velocity + 2;
        System.out.println("Velocity: " + velocity);
    }

    public void decrementVelocity() {
        velocity = velocity - 2;
        System.out.println("Velocity: " + velocity);

    }
    public void decrementFeed()
    {
        feedSpeed = feedSpeed - .05;
        System.out.println("Feed Speed: "+ feedSpeed);
    }
    
    public void incrementFeed()
    {
        feedSpeed = feedSpeed + .05;
        System.out.println("Feed Speed: "+ feedSpeed);
    }
    public Command set()
    {
        return run(()-> {
            double distance = drivebase.getDistanceToHub();

            //double velocity = getVelocity();
            double velocity = velocityMap.get(distance);
            System.out.println(distance);
            System.out.println("Velocity: " + velocity);
            if(velocity != 0)
            {
                controller1.setSetpoint(-velocity, SparkBase.ControlType.kVelocity);
                controller2.setSetpoint(-velocity, SparkBase.ControlType.kVelocity);
                controller3.setSetpoint(feedSpeed, SparkBase.ControlType.kDutyCycle);
                double indexSetpoint = 0.55;
                if(indexDirectionalityCounter < 0) {
                    indexSetpoint = -indexSetpoint;
                }
                if(indexDirectionalityCounter > indexDirectionalityMax) {
                    indexDirectionalityCounter = indexDirectionalityMin;
                }
                indexDirectionalityCounter++;
                indexController.setSetpoint(-indexSetpoint, SparkBase.ControlType.kDutyCycle);
            }
        });   
    }


    public double getVelocity()
    {
        double distance = drivebase.getDistanceToHub();
        System.out.println("Distance to Hub" + distance);
        if(distance == 0)
        {
            return 0;
        }
        return (17.7 * distance + 64);
    } 

   public void printDistance() {
        System.out.println("Distance: " + drivebase.getDistanceToHub());
   }
}
