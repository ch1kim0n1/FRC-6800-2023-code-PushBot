//You can tune the P gain to achieve the desired response of the arm motion. 
//Increasing the P gain will make the arm move faster but can also cause overshoot and oscillations. 
//The I and D gains can be added to improve the response and stability of the arm motion. 
//You can experiment with different values to find the best performance for your robot.

#include "Robot.h"
#include "RobotContainer.h"
#include "PID.h"

  //auto options
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOptions[] = { "NONE", "R_Left", "R_Mid", "R_Right", "IntakeBR", "IntakeGOBR", "TEST", "IntakeGO", "IntakeGOHARD"};



class Robot : public frc::TimedRobot {

  //4 main drive motor cotnrollers
    rev::CANSparkMax m_leftMotor1{1, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_leftMotor2{2, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightMotor1{3, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightMotor2{4, rev::CANSparkMax::MotorType::kBrushed};

    //combine left + right motors
    frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
    frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};

    //Combine both motor groups
    frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};

    //Controller for driver
    frc::XboxController controller{0}; 
    //Operator controller
    frc::XboxController controllerOP{1};

    //Timer for Auto
    frc::Timer m_timer;
    
    
 public:
  void RobotInit() override {

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);    

    //reset all motors before start
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();
    
    //list of Auto Options
    m_chooser.SetDefaultOption("NONE", kAutoOptions[0]);
    m_chooser.AddOption("R_Left", kAutoOptions[1]);
    m_chooser.AddOption("R_Mid", kAutoOptions[2]);
    m_chooser.AddOption("R_Right", kAutoOptions[3]);
    m_chooser.AddOption("IntakeBR", kAutoOptions[4]);
    m_chooser.AddOption("IntakeGOBR", kAutoOptions[5]);
    m_chooser.AddOption("TEST", kAutoOptions[6]);
    m_chooser.AddOption("IntakeGO", kAutoOptions[7]);
    m_chooser.AddOption("IntakeGOHARD", kAutoOptions[8]);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //limintation of speed and rotation sensitivity
    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 0.75);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 0.85);

    frc::SmartDashboard::PutNumber("Left Motor Limit", 1); //initially 65%
    frc::SmartDashboard::PutNumber("Right Motor Limit", 1);

    //limit motors to a specific voltage
    m_leftMotor1.EnableVoltageCompensation(12.3);
    m_leftMotor2.EnableVoltageCompensation(12.3);
    m_rightMotor1.EnableVoltageCompensation(12.3);
    m_rightMotor2.EnableVoltageCompensation(12.3);
    m_leftMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }

  void TeleopPeriodic() override {
    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.75);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 0.85);

    //numbers to limit both motor's capacity
    double sensLeft = frc::SmartDashboard::GetNumber("Left Motor Limit", 1); //65%? 0.65
    double sensRight = frc::SmartDashboard::GetNumber("Right Motor Limit", 1);


    //--------------------------------------DRIVE CODE HERE:--------------------------------------------
    //IMPORTANT: when mototr is - its going forward, and when its + its going backward (Its wierd but important for the auto)

    //motor speed and rotation variables from controller for ArcadeDrive
    double speed = controller.GetLeftY(); 
    double rotation = controller.GetRightX();
    //sqrt the results for the smoothest speed
    double SQRTspeed = sqrt(speed); 
    double SQRTrotation = sqrt(rotation);

    //main drive function 
    m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);
  }
  //*************************************************AUTONOMUS CODE*************************************************************
  void AutonomousInit() override {
    //check if those are working
    m_leftMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    double recangle = frc::SmartDashboard::GetNumber("Hor Angle", 0);

    //initial values for the auto
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    //inital speed of the intake is 40% for the sake of testing
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);
    
    //get the selected auto option
    std::string selectedOption = m_chooser.GetSelected();
    //convertion of double into seconds
    units::unit_t<units::time::second, double, units::linear_scale> secondsX(x);


  if(selectedOption == "R_Mid"){
    //MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
    else if(m_timer.Get() < 0.15_s + secondsX){
      m_robotDrive.TankDrive(0.6, 0.6, false); 
    }
    else if(m_timer.Get() < 1.3_s + secondsX){
      m_robotDrive.TankDrive(-0.7, -0.7, false); //initially 0.9 
    }
    else if(m_timer.Get() < 1.35_s + secondsX){
      m_robotDrive.TankDrive(0.8, 0.8, false); 
    }
    else if(m_timer.Get() < 1.7_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false); 
    }
    else{
      m_robotDrive.TankDrive(0, 0, false);
    }
  }

  else if (selectedOption == "TEST"){ //use with 12.5 voltage
     coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 3_s + secondsX){
    }
    else if(m_timer.Get() < 4_s + secondsX){
    }
    else if(m_timer.Get() < 4.5_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 6.5_s + secondsX){
      m_robotDrive.TankDrive(0.5, 0.5, false); //should go backwards
    }
     else if(m_timer.Get() < 9_s + secondsX){
       if(recangle > -70){
       m_robotDrive.TankDrive(-0.45, 0.45, false);
       }
       else{
         m_robotDrive.TankDrive(0, 0, false);
       }
     }
    else if(m_timer.Get() < 15_s + secondsX){ 
      m_robotDrive.TankDrive(0, 0, false);
    }
  }
  else if(selectedOption == "NONE"){
    if(m_timer.Get() < 1_s){
      m_robotDrive.TankDrive(0,0);
    }
    else if(m_timer.Get() < 15_s){
      m_robotDrive.TankDrive(0,0);
    }
    else{
      m_robotDrive.TankDrive(0,0);
    }
  }
  else if(selectedOption == "R_Left"){
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
    else if(m_timer.Get() < 0.15_s + secondsX){
      m_robotDrive.TankDrive(-0.6, -0.6, false); 
    }
    else if(m_timer.Get() < 1.5_s + secondsX){
      m_robotDrive.TankDrive(0.9, 0.9, false); //this is going between 0.15 and 1.2
    }
    else if(m_timer.Get() < 1.7_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false); 
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(-0.65, 0.65, false); 
    }
    else if(m_timer.Get() < 2.6_s + secondsX){ //0.3sec to turn 90 degrees with speed 0.62
      m_robotDrive.TankDrive(-0.6, -0.6, false); 
    }
    else if(m_timer.Get() < 3.1_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false); 
    }
    else if(m_timer.Get() < 3.4_s + secondsX){
      m_robotDrive.TankDrive(0.57, -0.57, false); 
    }
    else if(m_timer.Get() < 5_s + secondsX){
      m_robotDrive.TankDrive(-0.4, -0.4, false); //should go backwards
    }
    else{
      m_robotDrive.TankDrive(0, 0, false);
    }
  }

  else if(selectedOption == "IntakeBR"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 3_s + secondsX){
    }
    else if(m_timer.Get() < 4_s + secondsX){
    }
    else if(m_timer.Get() < 7_s + secondsX){
      m_robotDrive.TankDrive(0.4, 0.4, false); //should go backwards
    }
  }

  else if(selectedOption == "IntakeGOBR"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 3_s + secondsX){
    }
    else if(m_timer.Get() < 4_s + secondsX){
    }
    else if(m_timer.Get() < 7_s + secondsX){
      m_robotDrive.TankDrive(0.45, 0.45, false); //should go backwards
    }
    else{
      m_robotDrive.TankDrive(0, 0, false); 
    }
  }
  else if(selectedOption == "IntakeGO"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 3_s + secondsX){
    }
    else if(m_timer.Get() < 4_s + secondsX){
    }
    else if(m_timer.Get() < 5.5_s + secondsX){
      m_robotDrive.TankDrive(0.7, 0.7); //should go backwards
    }
    else if(m_timer.Get() < 15_s + secondsX){
      m_robotDrive.TankDrive(0, 0);
    }
    else{
      m_robotDrive.TankDrive(0, 0);
    }
  }
  else if(selectedOption == "IntakeGOHARD"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 3_s + secondsX){
    }
    else if(m_timer.Get() < 4_s + secondsX){
    }
    else if(m_timer.Get() < 6_s + secondsX){
      m_robotDrive.TankDrive(0.7, 0.7); //should go backwards
    }
    else if(m_timer.Get() < 15_s + secondsX){
      m_robotDrive.TankDrive(0, 0);
    }
    else{
      m_robotDrive.TankDrive(0, 0);
    }
  }
}
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif