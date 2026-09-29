package frc.robot.subsystems.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
    private final double raiseSpeed = .10;

    // Set IDs
    SparkMax raiseMax1 = new SparkMax(61, MotorType.kBrushless);
    SparkMax raiseMax2 = new SparkMax(62, MotorType.kBrushless);
    SparkMax intakeMax;// = new SparkMax(61, MotorType.kBrushless);

    private final RelativeEncoder raiseEncoder1;
    private final RelativeEncoder raiseEncoder2;
    private final RelativeEncoder intakeEncoder;

    private  SparkClosedLoopController intakeController;
    private final SparkClosedLoopController raiseController1;
    private final SparkClosedLoopController raiseController2;

    private final SparkMaxConfig intakeConfig;
    private final SparkMaxConfig raiseConfig;

    public Intake()
    {
        raiseEncoder1 = raiseMax1.getEncoder();
        raiseEncoder2 = raiseMax2.getEncoder();
        intakeEncoder = intakeMax.getEncoder();

        //intakeController = intakeMax.getClosedLoopController();
        raiseController1 = raiseMax1.getClosedLoopController();
        raiseController2 = raiseMax2.getClosedLoopController();

        intakeConfig = new SparkMaxConfig();
        raiseConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kCoast);
        raiseConfig.idleMode(IdleMode.kBrake);


        raiseMax1.configure(raiseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        raiseMax2.configure(raiseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMax.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        setDefaultCommand(
             runOnce(
                     () -> {
                        //intakeController.setSetpoint(0, SparkBase.ControlType.kDutyCycle);
                        raiseController1.setSetpoint(0, SparkBase.ControlType.kDutyCycle);
                        raiseController2.setSetpoint(0, SparkBase.ControlType.kDutyCycle);
                    
                     })
                 .andThen(run(() -> {}))
                 .withName("Idle"));

    }
    public Command raiseIntake()
    {
        return run(()-> {
            //Use Negative Setpoints
            //returns to top
            raiseController1.setSetpoint(0.0, SparkBase.ControlType.kPosition);
            raiseController2.setSetpoint(0.0, SparkBase.ControlType.kPosition);
        });
    }
    public Command lowerIntake()
    {
        return run(()-> {
            //Use Positive Setpoints
            //Need to set position points
            raiseController1.setSetpoint(0.0, SparkBase.ControlType.kPosition);
            raiseController2.setSetpoint(0.0, SparkBase.ControlType.kPosition);
        });
    }
    public Command manualRaiseIntake()
    {
        return run(()-> {
            //Use Negative Setpoints
            System.out.println("^");
            System.out.println("|");
            raiseController1.setSetpoint(-raiseSpeed, SparkBase.ControlType.kDutyCycle);
            raiseController2.setSetpoint(raiseSpeed / 8, SparkBase.ControlType.kDutyCycle);
        });
    }
    public Command manualLowerIntake()
    {
        return run(()-> {
            //Use Positive Setpoints
            System.out.println("|");
            System.out.println("v");
            System.out.println("Position" + raiseEncoder1.getPosition());
            raiseController1.setSetpoint(raiseSpeed, SparkBase.ControlType.kDutyCycle);
            raiseController2.setSetpoint(-raiseSpeed / 8, SparkBase.ControlType.kDutyCycle);
        });
    }
    public Command runIntake()
    {
        return run(()->{
            //Use Negative Setpoints
            System.out.println("Intaking");

            intakeController.setSetpoint(-0.65, SparkBase.ControlType.kDutyCycle);
        }
            
        );
    }
}