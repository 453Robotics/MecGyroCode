package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a sample program that uses mecanum drive with a gyro sensor to
 * maintain rotation vectorsin relation to the starting orientation of the robot
 * (field-oriented controls).
 */
public class Robot extends TimedRobot {
  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 3;
  private static final int kRearRightChannel = 4;

  private static final CANSparkMax frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
  private static final CANSparkMax rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
  private static final CANSparkMax frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
  private static final CANSparkMax rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);

  private static final CANEncoder m_frontLeftEncoder = frontLeft.getEncoder();
  private static final CANEncoder m_frontRightEncoder = frontRight.getEncoder();
  private static final CANEncoder m_rearLeftEncoder = rearLeft.getEncoder();
  private static final CANEncoder m_rearRightEncoder = rearRight.getEncoder();

  private static final int kGyroPort = 0;
  private static final int kJoystickPort = 0;

  private static final double wheelDiam = 6.0;

  private MecanumDrive m_robotDrive;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  double countsPerRev = m_frontLeftEncoder.getCountsPerRevolution();
  double gearRatio = 16.0;

  // The gain for a simple P loop
  double kP = 1;

  // The heading of the robot when starting the motion
  double heading;

  @Override
  public void robotInit() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);

    SmartDashboard.putNumber("FrontLeft Distance (RAW)", m_frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("FrontLeft Distance (Inches)",
        (m_frontLeftEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam);
    SmartDashboard.putNumber("FrontRight Distance (RAW)", m_frontRightEncoder.getPosition());
    SmartDashboard.putNumber("FrontRight Distance (Inches)",
        (m_frontRightEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam);
    SmartDashboard.putNumber("RearLeft Distance (RAW)", m_rearLeftEncoder.getPosition());
    SmartDashboard.putNumber("RearLeft Distance (Inches)",
        (m_rearLeftEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam);
    SmartDashboard.putNumber("RearRight Distance (RAW)", m_rearRightEncoder.getPosition());
    SmartDashboard.putNumber("RearRight Distance (Inches)",
        (m_rearRightEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam);

  }

  @Override
  public void autonomousInit() {
    // Set setpoint to current heading at start of auto
    heading = m_gyro.getAngle();
  }

  @Override
  public void autonomousPeriodic() {
    driveStrtXDist(60);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    turnXDeg(90);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    driveStrtXDist(60);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    turnXDeg(90);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    driveStrtXDist(60);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    turnXDeg(90);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    driveStrtXDist(60);
    motorStop();
    try {
      wait(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    turnXDeg(90);
    while(true){
      motorStop();
    }
  }

  /** Mecanum drive is used with the gyro angle as an input. */
  @Override
  public void teleopPeriodic() {
    
    m_robotDrive.driveCartesian(
        m_joystick.getRawAxis(0), m_joystick.getRawAxis(1), m_joystick.getRawAxis(4), m_gyro.getAngle());

    SmartDashboard.updateValues();
  }

  public void turnXDeg(double targetAngle){
    m_gyro.reset();
    if(targetAngle < 0){
      while(m_gyro.getAngle() > targetAngle){
        m_robotDrive.driveCartesian(0, 0, -.2, 0);
      }
    }
    if(targetAngle > 0){
      while(m_gyro.getAngle() < targetAngle){
        m_robotDrive.driveCartesian(0, 0, .2, 0);
      }
    }
    
  }

  public void driveStrtXDist(double targetDist){
    m_gyro.reset();
    if(targetDist < 0){
      while((m_frontLeftEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam > targetDist){
        double error = heading - m_gyro.getAngle();
        m_robotDrive.driveCartesian(.2, 0, kP * error, 0);
      }
    }
    if(targetDist > 0){
      while((m_frontLeftEncoder.getPosition() / gearRatio) * Math.PI * wheelDiam < targetDist){
        double error = heading - m_gyro.getAngle();
        m_robotDrive.driveCartesian(-.2, 0, -kP * error, 0);
      }
    }
  }

  public void motorStop(){
    m_robotDrive.driveCartesian(0, 0, 0, 0);
  }
}
