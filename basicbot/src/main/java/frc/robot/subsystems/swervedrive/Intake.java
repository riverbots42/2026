package frc.robot.subsystems.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {

    // Set IDs
    SparkMax raiseMax = new SparkMax(61, MotorType.kBrushless);
    SparkMax intakeMax = new SparkMax(62, MotorType.kBrushless);

    private final RelativeEncoder raiseEncoder;
    private final RelativeEncoder intakeEncoder;

    private final SparkClosedLoopController intakeController;
    private final SparkClosedLoopController raiseController;

    private final SparkMaxConfig config;

    public Intake()
    {
        raiseEncoder = raiseMax.getEncoder();
        intakeEncoder = intakeMax.getEncoder();

        intakeController = intakeMax.getClosedLoopController();
        raiseController = raiseMax.getClosedLoopController();

        config = new SparkMaxConfig();

        setDefaultCommand(
             runOnce(
                     () -> {
                        intakeController.setSetpoint(0, SparkBase.ControlType.kDutyCycle);
                        raiseController.setSetpoint(0, SparkBase.ControlType.kDutyCycle);
                    
                     })
                 .andThen(run(() -> {}))
                 .withName("Idle"));

    }
    public Command raiseIntake()
    {
        return run(()-> {
            //Use Negative Setpoints
            //returns to top
            raiseController.setSetpoint(0.0, SparkBase.ControlType.kPosition);
        });
    }
    public Command lowerIntake()
    {
        return run(()-> {
            //Use Positive Setpoints
            //Need to set position points
            raiseController.setSetpoint(0.0, SparkBase.ControlType.kPosition);
        });
    }
    public Command manualRaiseIntake()
    {
        return run(()-> {
            //Use Negative Setpoints
            System.out.println("^");
            System.out.println("|");
            raiseController.setSetpoint(-0.10, SparkBase.ControlType.kDutyCycle);
        });
    }
    public Command manualLowerIntake()
    {
        return run(()-> {
            //Use Positive Setpoints
            System.out.println("|");
            System.out.println("v");
            raiseController.setSetpoint(0.10, SparkBase.ControlType.kDutyCycle);
        });
    }
    public Command runIntake()
    {
        return run(()->{
            //Use Negative Setpoints
            // add correct percentage
            System.out.println("yo mama");

            intakeController.setSetpoint(-0.15, SparkBase.ControlType.kDutyCycle);
        }
            
        );
    }
}
