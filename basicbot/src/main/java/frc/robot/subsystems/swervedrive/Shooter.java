package frc.robot.subsystems.swervedrive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.swervedrive.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;


public class Shooter extends SubsystemBase
{
    SparkMax spinnerMax1 = new SparkMax(40, MotorType.kBrushless);
    SparkMax spinnerMax2 = new SparkMax(41, MotorType.kBrushless);
    SparkMax feederMax = new SparkMax(39, MotorType.kBrushless);

    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final RelativeEncoder feederEncoder;

    private final SparkClosedLoopController controller1;
    private final SparkClosedLoopController controller2;
    private final SparkClosedLoopController controller3;

    private final double workingKf;
    private final SparkMaxConfig config;
    private Vision vision;
    private PhotonPipelineResult latestResult;

    public Shooter()
    {
        encoder1 = spinnerMax1.getEncoder();
        encoder2 = spinnerMax2.getEncoder();
        feederEncoder = feederMax.getEncoder();
        controller1 = spinnerMax1.getClosedLoopController();
        controller2 = spinnerMax2.getClosedLoopController();
        controller3 = feederMax.getClosedLoopController();
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        workingKf = -0.511905;
        //config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.002064500004053116,0.0,0.0);
        //config.encoder.positionConversionFactor(0.);
        //shoot.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        setDefaultCommand(
             runOnce(
                     () -> {
                        controller1.setSetpoint(0, SparkBase.ControlType.kVelocity);
                        controller2.setSetpoint(0, SparkBase.ControlType.kVelocity);
                        controller3.setSetpoint(0, SparkBase.ControlType.kVelocity);
                     })
                 .andThen(run(() -> {}))
                 .withName("Idle"));

    }
    public void set()
    {
        //return run(() -> {
            //System.out.println("Distance: " + distance);
            //double velocity = getVelocity(distance);
            double velocity = 150;
            System.out.println("Velocity:" + velocity);
            System.out.println("Encoder Velocity: " + encoder1.getVelocity());

            if(velocity != 0)
            {
                controller1.setSetpoint(-velocity, SparkBase.ControlType.kVelocity);
                controller2.setSetpoint(-velocity, SparkBase.ControlType.kVelocity);
                controller3.setSetpoint(-100, SparkBase.ControlType.kVelocity);
            }
            
      //  });
    }

    public double getVelocity(double distance)
    {
        if(distance == 0)
        {
            return 0;
        }
        return (workingKf * distance -106.52381);
    } 

    /*public Command feed()
    {
        return run(() -> {controller3.setSetpoint(-50, SparkBase.ControlType.kVelocity);});
    } */

}
