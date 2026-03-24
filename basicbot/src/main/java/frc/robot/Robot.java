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
  //Blue Autos
  private static final String kTestAuto = "Test; Shoot";
  private static final String kBShootLeft = "Blue 1; Shoot";
  private static final String kBShootCenter = "Blue 2; Shoot";
  private static final String kBShootRight = "Blue 3; Shoot";
  private static final String kBIntakeLeftT = "Blue 1; Intake Trench";
  private static final String kBIntakeRightT = "Blue 3; Intake Trench";
  private static final String kBIntakeRightB = "Blue 3; Intake Bump";
  private static final String kBIntakeLeftB = "Blue 1; Intake Bump";
  //private static final String kShootCenter2 = "Auto 7; Shoot";
  
  //Red Autos
  private static final String kRShootLeft = "Red 1; Shoot";
  private static final String kRShootCenter = "Red 2; Shoot";
  private static final String kRShootRight = "Red 3; Shoot";
  private static final String kRIntakeLeftT = "Red 1; Intake Trench";
  private static final String kRIntakeRightT = "Red 3; Intake Trench";
  private static final String kRShootPF4 = "Red 1; Shoot PF 4";

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
   
    
    
    //Chooses Blue Alliance Autos
    
    m_chooser.addOption("B1 Shoot Left", kBShootLeft);
    m_chooser.addOption("B2 Shoot Center", kBShootCenter);
    m_chooser.addOption("B3 Shoot Right", kBShootRight);
    m_chooser.addOption("B1 Intake Trench", kBIntakeLeftT);
    m_chooser.addOption("B3 Intake Trench", kBIntakeRightT);
    m_chooser.addOption("B1 Intake Bump", kBIntakeLeftB);
    m_chooser.addOption("B3 Intake Bump", kBIntakeRightB);
    //m_chooser.addOption("Eat Balls", kAuto1Eat);

    //Chooses Red Alliance Autos
    m_chooser.addOption("R1 Shoot Left", kRShootLeft);
    m_chooser.addOption("R2 Shoot Center", kRShootCenter);
    m_chooser.addOption("R3 Shoot Right", kRShootRight);
    m_chooser.addOption("R1 Intake Trench", kRIntakeLeftT);
    m_chooser.addOption("R3 Intake Trench", kRIntakeRightT);
    m_chooser.addOption("R1 Shoot PF 4", kRShootPF4);

    m_chooser.setDefaultOption("Test Auto", kTestAuto);
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
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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
    // if(isRedAlliance())
    // {
    //   switch(m_chooser.getSelected())
    //   {
    //     case kBShootLeft:
    //       m_autoSelected = kRShootLeft;
    //       break;
    //     case kBShootCenter:
    //       m_autoSelected = kRShootCenter;
    //       break;
    //       case kBShootRight:
    //       m_autoSelected = kRShootRight;
    //       break;
    //     case kBIntakeLeftT:
    //       m_autoSelected = kRIntakeLeftT;
    //       break;
    //     case kBIntakeRightT:
    //       m_autoSelected = kRIntakeRightT;
    //       break;
    //     case kBIntakeRightB:
    //       m_autoSelected = null;
    //       break;
    //     case kBIntakeLeftB:
    //       m_autoSelected = null;
    //       break;
          
    //   }
    // }
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
