/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.EnableFalconVelocityClosedLoop;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FalconClosedLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private Joystick toggler = new Joystick(0);
  public class VictorWrapper extends SubsystemBase {
    private VictorSPX VictorController;
    private double rateControlled;
    private double t = 0.0;
    public VictorWrapper(int VictorChannel) {
      this.VictorController = new VictorSPX(VictorChannel);
      this.rateControlled = 0.0;
    }

    public void incrementPower(double increment) {
      this.rateControlled += increment;
      this.VictorController.set(ControlMode.PercentOutput,rateControlled/192307);
    }
  }
  private Encoder enc1 = new Encoder(0, 1);
  private FalconClosedLoop Cim1 = new FalconClosedLoop(7,0,30,ControlMode.Velocity);
  private EnableFalconVelocityClosedLoop VelCL1 = new EnableFalconVelocityClosedLoop(Cim1,10000);

  private Encoder enc2 = new Encoder(2, 3);
  private FalconClosedLoop Cim2 = new FalconClosedLoop(6,0,30,ControlMode.Velocity);
  private EnableFalconVelocityClosedLoop VelCL2 = new EnableFalconVelocityClosedLoop(Cim2,-10000);

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
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
   */
  public long startingTime = 0;
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    startingTime = java.lang.System.currentTimeMillis();
    PIDController PIDUsed = new PIDController(.1,0.00001,1,20);
    PIDUsed.enableContinuousInput(-192307, 192307);
    return new ParallelCommandGroup(VelCL2,VelCL1);//new PIDCommand(PIDUsed, () -> enc1.getRate(), 100000, (double output) -> {
      //Cim1.incrementPower((output));
      //System.out.println(((Long) (java.lang.System.currentTimeMillis()-startingTime)).toString() + ((Double) enc1.getRate()).toString());
    //},Cim1),VelCL);
  }
}
