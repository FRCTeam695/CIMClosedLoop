/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.annotation.JsonPropertyDescription;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private class MotorPercent extends CommandBase {
    private HashMap<EnableFalconVelocityClosedLoop,Double> LoopToPercent = new HashMap<EnableFalconVelocityClosedLoop,Double>();
    private EnableFalconVelocityClosedLoop topLoop,bottomLoop;
    private boolean printOut = true;
    private TurretMotor Motor;
    MotorPercent (EnableFalconVelocityClosedLoop topLoop, double percentTop,EnableFalconVelocityClosedLoop bottomLoop, double percentBottom,TurretMotor Motor) {
      LoopToPercent.put(topLoop, percentTop);
      LoopToPercent.put(bottomLoop, percentBottom);
      this.topLoop = topLoop;
      this.bottomLoop = bottomLoop;
      this.Motor = Motor;
    }

    public void changePercent(EnableFalconVelocityClosedLoop Loop, double newPercent) {
      if (LoopToPercent.get(Loop) != null) {
        LoopToPercent.put(Loop,newPercent);
      }
    }

    public void execute() {
      if (printOut) {
        System.out.println("TOP: " + LoopToPercent.get(topLoop).toString() + " Bottom: " + LoopToPercent.get(bottomLoop).toString() + "LIME DISTANCE:" + ((Double) Motor.getDistanceToContour()).toString());
      }
    }

    public void setEnabled(boolean newState) {
      printOut = newState;
    }
    public void toggleEnabled() {
      setEnabled(!printOut);
    }
  }
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private Joystick toggler = new Joystick(0);
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();

  private final TurretMotor Turret = new TurretMotor(RobotMainNetworkTableInstance, 8);
  private final AutoTurretRotation Finding = new AutoTurretRotation(Turret);
  private final AutoTurretFocus Focusing = new AutoTurretFocus(Turret);
  private final TurretFocusPID FocusingPID = new TurretFocusPID(Turret);
  private final SequentialCommandGroup TurretGroup = new SequentialCommandGroup(Finding,FocusingPID);

  private double topPercent = 0;//0.8;
  private double bottomPercent = 0;//-0.15;

  private Encoder enc1 = new Encoder(0, 1);
  //7 is top, 6 is bottom
  private CimClosedLoop TopCIM = new CimClosedLoop(7,0,30,ControlMode.Velocity);
  //private EnableFalconVelocityClosedLoop VelCLTOP = new EnableFalconVelocityClosedLoop(Cim1,10000*topPercent);

  private Encoder enc2 = new Encoder(2, 3);
  private CimClosedLoop BottomCIM = new CimClosedLoop(6,0,30,ControlMode.Velocity);
  //private EnableFalconVelocityClosedLoop VelCLBOTTOM = new EnableFalconVelocityClosedLoop(Cim2,10000*bottomPercent);

  //private final MotorPercent MotorTracker = new MotorPercent(VelCLTOP, topPercent, VelCLBOTTOM, bottomPercent,Turret);

  //private ParallelCommandGroup AutonGroup = new ParallelCommandGroup(VelCLTOP,VelCLBOTTOM,MotorTracker);
  private Joystick ControllerDrive = new Joystick(0);
  private final POVButton POVTop= new POVButton(ControllerDrive, 0);
  private final POVButton POVBottom= new POVButton(ControllerDrive, 180);
  private final JoystickButton YButton = new JoystickButton(ControllerDrive, 4);
  private final JoystickButton AButton = new JoystickButton(ControllerDrive, 1); 
  private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  private final JoystickButton BButton = new JoystickButton(ControllerDrive, 2); 
  private final JoystickButton ShoulderLeft = new JoystickButton(ControllerDrive, 5);
  private final JoystickButton ShoulderRight = new JoystickButton(ControllerDrive, 6);
  private final JoystickButton LJoystickClick = new JoystickButton(ControllerDrive, 9);
  private final JoystickButton RJoystickClick = new JoystickButton(ControllerDrive, 10);

  private double adjustAmount = 0.05;

  private void setLoopPowerPercent(EnableFalconVelocityClosedLoop Loop,double percent) {
    System.out.println(percent);
    Loop.changeVelocity(percent*10000);
    MotorTracker.changePercent(Loop, percent);
  }
  //8 - dio 0,1
  //5- dio 2,3
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  /** 
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    BButton.whenPressed(TurretGroup);
    XButton.whenPressed(new InstantCommand(Focusing::change));
    POVTop.whenPressed(() -> setLoopPowerPercent(VelCLTOP,topPercent+=adjustAmount));
    POVBottom.whenPressed(() -> setLoopPowerPercent(VelCLTOP,topPercent-=adjustAmount));

    YButton.whenPressed(() -> setLoopPowerPercent(VelCLBOTTOM,bottomPercent-=adjustAmount));
    AButton.whenPressed(() -> setLoopPowerPercent(VelCLBOTTOM,bottomPercent+=adjustAmount));
    ShoulderLeft.whenPressed(() -> {adjustAmount = 0.01;}); //fine adjust
    ShoulderLeft.whenReleased(() -> {adjustAmount = 0.05;}); //return to coarse
    ShoulderRight.whenPressed(() -> {adjustAmount = 0.001;}); //extra fine adjust
    ShoulderRight.whenReleased(() -> {adjustAmount = 0.05;}); //return to coarse`
    LJoystickClick.whenPressed(MotorTracker::toggleEnabled); //toggle prints
    RJoystickClick.whenPressed(() -> {setLoopPowerPercent(VelCLTOP,topPercent=0);setLoopPowerPercent(VelCLBOTTOM,bottomPercent=0);});
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
   */
  public long startingTime = 0;
  public Command getAutonomousCommand() {
    System.out.println(topPercent);
    System.out.println(bottomPercent);
    // An ExampleCommand will run in autonomous
    //startingTime = java.lang.System.currentTimeMillis();
    //PIDController PIDUsed = new PIDController(.1,0.00001,1,20);
    //PIDUsed.enableContinuousInput(-192307, 192307);
    return AutonGroup;//new PIDCommand(PIDUsed, () -> enc1.getRate(), 100000, (double output) -> {
      //Cim1.incrementPower((output));
      //System.out.println(((Long) (java.lang.System.currentTimeMillis()-startingTime)).toString() + ((Double) enc1.getRate()).toString());
    //},Cim1),VelCL);
  }
  public Command getTeleopCommand() {
    System.out.println(topPercent);
    System.out.println(bottomPercent);
    return AutonGroup;
  }
}
