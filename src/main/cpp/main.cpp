
#include <frc/Joystick.h>

#include <frc/TimedRobot.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include <frc/Timer.h>
#include <iostream>

using namespace std;


class Helen_Test : public frc::TimedRobot {

  /**

   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object

   * 

   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the

   * first parameter

   * 

   * The motor type is passed as the second parameter. Motor type can either be:

   *  rev::CANSparkMax::MotorType::kBrushless

   *  rev::CANSparkMax::MotorType::kBrushed

   * 

   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change

   * these parameters to match your setup

   */

  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;

  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder leftLead_encoder = m_leftLeadMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder rightLead_encoder = m_rightLeadMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder leftFollower_encoder = m_leftFollowMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder rightFollower_encoder = m_rightFollowMotor.GetEncoder();

  float rightSpeed = 0.6;

  float leftSpeed = 0.6;

  bool hardStop = false;

  vector<int> speeds;
  /**

   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 

   * m_leftLeadMotor and m_rightLeadMotor, respectively. 

   * 

   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 

   * sent to them will automatically be copied by the follower motors

   */

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

 private:
    // static void VisionThread(){
        
    //     cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    //     camera.SetResolution(320,240);
    //     cs::CvSink cvSink = frc::CameraServer::GetVideo();
    //     // cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 320, 240);
    //     cv::Mat mat;
    //     cv::Mat grayMat;
    //     frc::AprilTagDetector aprilTagDetector;
    //     aprilTagDetector.AddFamily("tag36h11", 0);
    //     cv::Size g_size = grayMat.size();
    //     bool stop = false;
      

        // while(true){
            
        //     // if(cvSink.GrabFrame(mat) == 0){
        //     //     outputStream.NotifyError(cvSink.GetError());
        //     //     std::cout << "BIG DOO DOO" << std::endl;
        //     //     continue;
        //     // }
        //     cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);
        //     frc::AprilTagDetector::Results detections = aprilTagDetector.Detect(g_size.width, g_size.height, grayMat.data);
        //     for (const frc::AprilTagDetection* detection : detections) {
        //         frc::SmartDashboard::PutBoolean("stop", true);
        //     }
        //     // outputStream.PutFrame(mat);
        // }
    // }

 public:




 
    Helen_Test() {}

    void RobotInit() override{
        m_rightFollowMotor.Follow(m_rightLeadMotor, false);
        m_leftFollowMotor.Follow(m_leftLeadMotor, false);
        m_rightLeadMotor.SetInverted(true);

        // std::thread visionThread(VisionThread);
        // visionThread.detach();
    }

    void AutonomousPeriodic() override {
        // drive(200);
        turn(90);
        // drive(10);
        // bool stop = frc::SmartDashboard::GetBoolean("stop", false);
        // if (stop){
            
        //     // rightSpeed = 0.0;
        //     // leftSpeed = 0.0;
        // }
        // std::cout << stop << std::endl;
        // m_rightLeadMotor.Set(rightSpeed);
        // m_leftLeadMotor.Set(leftSpeed);
        
    }

    void turn(float angle) {
        float encoder_goal = (angle/360)*(3.14*10) * 42;
        float speed = 0.5;
        while(leftLead_encoder.GetPosition()<= encoder_goal || abs(rightLead_encoder.GetPosition())<= encoder_goal) {
            int errorLeft = encoder_goal - leftLead_encoder.GetPosition();
            int errorRight = encoder_goal - abs(rightLead_encoder.GetPosition());
            int k = 0.2;

            m_leftLeadMotor.Set(speed - (errorLeft * k));
            m_rightLeadMotor.Set(-speed - (errorRight * k));
        }
            m_leftLeadMotor.Set(0);
            m_rightLeadMotor.Set(0);
    } 

    void drive (float distance) {
        float encoder_goal = distance/(3.14*10) * 42;
        float speed = 0.3;
        while(leftLead_encoder.GetPosition() <= encoder_goal || rightLead_encoder.GetPosition() <= encoder_goal) {
            int errorLeft = encoder_goal - leftLead_encoder.GetPosition();
            int errorRight = encoder_goal - rightLead_encoder.GetPosition();
            int k = 0.2;

            m_leftLeadMotor.Set(speed - (errorLeft * k));
            m_rightLeadMotor.Set(speed - (errorRight * k));

        }
            m_leftLeadMotor.Set(0);
            m_rightLeadMotor.Set(0);

    }

    void print_encoders() {
        frc::SmartDashboard::PutNumber("Left Lead Encoder Position", leftLead_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Right Lead Position", rightLead_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Left Follower Position", leftFollower_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Right Follower Position", rightLead_encoder.GetPosition());

    }
};

    // Helen_Test() {

    //     m_timer.Start();

    // }



    // void AutonomousPeriodic() override {

    //     if(m_timer.Get() < 2_s) {

    //         m_robotDrive.ArcadeDrive(0.5,0.0,false);

    //     } else{ 

    //         m_robotDrive.ArcadeDrive(0.0,0.0, false);

    //     }

    // }



    // private:

    // frc::Timer m_timer;







#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Helen_Test>(); }
#endif



