// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kTestAuto = "Test Auto";
  private static final String kAuto1 = "Auto 1;Shoot";
  private static final String kAuto2 = "Auto 2;Shoot";
  private static final String kAuto3 = "Auto 3;Shoot";
  private static final String kAuto1Eat = "Auto 1; Intake Balls";

  private final Field2d m_field = new Field2d();

  private String m_autoSelected;
  private Command m_autoCommand;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
   
    m_chooser.setDefaultOption("Test Auto", kTestAuto);
    m_chooser.addOption("Auto 1", kAuto1);
    m_chooser.addOption("Auto 2", kAuto2);
    m_chooser.addOption("Auto 3", kAuto3);
    m_chooser.addOption("Eat Balls", kAuto1Eat);

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

/**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    System.out.println("Robot Init; Making Container");
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_field.setRobotPose(m_robotContainer.getSwerve().getPose());
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Start Of Auto Init; Chooser Selecting");
    m_autoSelected = m_chooser.getSelected();
    m_robotContainer.setupPathPlannerThroughTheThang();
    System.out.println("Set it up");
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    if(m_autoSelected != null)
    {
      System.out.println("Loading selected auto");
      PathPlannerAuto pathPlannerAuto = new PathPlannerAuto(m_autoSelected, isRedAlliance());
      m_autoCommand = pathPlannerAuto;
      CommandScheduler.getInstance().schedule(m_autoCommand);
    }
    else{
      System.out.println("Auto selected is poop");
    }
    System.out.println("Auto selected: " + m_autoSelected);
  }
  public boolean isRedAlliance()
  {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      return true;
    }
    return false;
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    //}
    

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
