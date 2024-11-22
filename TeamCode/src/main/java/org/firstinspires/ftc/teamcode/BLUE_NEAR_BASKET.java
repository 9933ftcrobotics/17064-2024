package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BLUE_NEAR_BASKET", group = "")
//@Disabled
public class BLUE_NEAR_BASKET extends LinearOpMode {

  BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
  private DcMotor FrontRight;
  private DcMotor FrontLeft;
  private DcMotor RearRight;
  private DcMotor RearLeft;
  private DcMotor ArmExtender;
  private DcMotor arm;
  private CRServo InTake;

  int Ticks = 0;
  int Ticks2 = 0;
  int Distance2;
  double Speed = 0.65;

//  double TICKS_PER_INCH = 39.76;
  double TICKS_PER_INCH = 42;



  @Override
  public void runOpMode() {




    //Put Motor Name = dcMotor Motor Name here 
   FrontRight=hardwareMap.dcMotor.get("RightFront");
   FrontLeft=hardwareMap.dcMotor.get("LeftFront");
   RearRight=hardwareMap.dcMotor.get("RearRight");
   RearLeft=hardwareMap.dcMotor.get("RearLeft");
   InTake = hardwareMap.get(CRServo.class, "InTake");
   ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
   arm = hardwareMap.get(DcMotor.class, "ArmLift");

  FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
  RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
  ArmExtender.setDirection(DcMotor.Direction.REVERSE);

  FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  ArmExtender.setTargetPosition(0);
  ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  arm.setTargetPosition(0);
  arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

  FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
      parameters.loggingEnabled      = true;
      parameters.loggingTag          = "IMU";
      parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

  imu = hardwareMap.get(BNO055IMU.class, "imu");
  imu.initialize(parameters);


    telemetry.addData(">", "Press Play to start op mode");
    telemetry.update(); 
    waitForStart();
    if (opModeIsActive()) {
        arm.setPower(1);
        ArmExtender.setPower(0.8);



        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.addData("Ticks:",Ticks);
      telemetry.update();
      telemetry.addData("Ticks2:",Ticks2);
      telemetry.update();
      //////////////////////////////////////////////////////////////////////////////////
      //        Create list of Commands for the Auto here
      //////////////////////////////////////////////////////////////////////////////////
      Speed = 0.35;

      //Drive(27.5,Speed);
      Drive(8,Speed);
      Turn(-90,1);
        arm.setTargetPosition(650);
        Drive(6,Speed);
        ArmExtender.setTargetPosition(1900);
        Drive(6,Speed);
        arm.setTargetPosition(800);
        sleep(1000);
        Turn(-135,1);
        Drive(6,Speed);
        InTake.setPower(1);
        sleep(3000);
        arm.setTargetPosition(890);
        sleep(1000);
        Drive(-6,Speed);
        InTake.setPower(0);
        //Drive(-16,Speed);
        Turn(0,1);
        InTake.setPower(0);
        arm.setPower(0.2); //new
        arm.setTargetPosition(650);
        ArmExtender.setTargetPosition(0);
        arm.setTargetPosition(450);
        sleep(1500);
      //////////////////////////////////////////////////////////////////////////////////
    }
  }

   private void initializeGyro() {
          BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
          parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
          parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
          parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
          parameters.loggingEnabled      = true;
          parameters.loggingTag          = "IMU";        
          parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            telemetry.addLine("Gyro Calibrating...");
            telemetry.update();
        } 
   }


    private void Turn(int Angle, double Power) 
{     
      //TURN RIGHT IS POSITIVE
      //TURN LEFT IS NEGATIVE

      telemetry.addLine("We are now inside Turn function");
      telemetry.update();

      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      float RobotAngle = -angles.firstAngle;
      float StartingAngle = RobotAngle;
      boolean StartFast = false;
      telemetry.addData("Gyro angle", RobotAngle);
      telemetry.update();

      FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      float TotalAngle = Math.abs(RobotAngle-Angle);
      float StartSlowingDownAngleOffset = 0;

      if (TotalAngle < 40){        
        StartFast = false;
      }
      else if (TotalAngle > 90)
      {
        StartSlowingDownAngleOffset = (TotalAngle-90)/9;
      }
      else
      {      
        StartFast = true;
      }

      if (Angle < RobotAngle) {
        while ((Angle < RobotAngle - (75+StartSlowingDownAngleOffset)) || (((StartingAngle - RobotAngle)  < 1) && StartFast))
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(-Power);
            FrontRight.setPower(Power);
            RearRight.setPower(Power);
            RearLeft.setPower(-Power);
          }
        while (Angle < RobotAngle-2.5)
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(-0.25);
            FrontRight.setPower(0.25);
            RearRight.setPower(0.25);
            RearLeft.setPower(-0.25);
          }



          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotAngle = -angles.firstAngle;


      }



       else {

          while ((Angle - (75+StartSlowingDownAngleOffset) > RobotAngle) || (((RobotAngle-StartingAngle)  < 1) && StartFast))
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(Power);
            FrontRight.setPower(-Power);
            RearRight.setPower(-Power);
            RearLeft.setPower(Power);
          }
          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

          while (Angle - 2.5 > RobotAngle)
          {
            telemetry.addData("Gyro",RobotAngle );
            telemetry.addData("TotalAngle",TotalAngle );
            telemetry.addData("StartSlowingDownAngleOffset",StartSlowingDownAngleOffset );
            telemetry.update();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            RobotAngle = -angles.firstAngle;
            FrontLeft.setPower(0.25);
            FrontRight.setPower(-0.25);
            RearRight.setPower(-0.25);
            RearLeft.setPower(0.25);
          }
          FrontLeft.setPower(0);
          FrontRight.setPower(0);
          RearRight.setPower(0);
          RearLeft.setPower(0);
       }


}


    private void Drive(double Distance, double Power) {
      //Distance2 = Distance * 50;
 //     Ticks = Distance2 - 50;
      Ticks = (int) (TICKS_PER_INCH * Distance);

      //sleep(100);


      FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + Ticks);
      FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + Ticks);
      RearRight.setTargetPosition(RearRight.getCurrentPosition() + Ticks);
      RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + Ticks);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setPower(Power);
      FrontLeft.setPower(Power);
      RearRight.setPower(Power);
      RearLeft.setPower(Power);
      while(FrontRight.isBusy())
      {
        telemetry.addData("CPosition",FrontRight.getCurrentPosition() );
        telemetry.update();
        if(isStopRequested() || !opModeIsActive()){
          break;
        }
      }
      FrontRight.setPower(0);
      FrontLeft.setPower(0);
      RearRight.setPower(0);
      RearLeft.setPower(0);

      sleep(100);
      FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

  private void Drive_Simple(int Time, double Power) {
    
    FrontRight.setPower(Power);
    FrontLeft.setPower(Power);
    RearRight.setPower(Power);
    RearLeft.setPower(Power);

    sleep(Time);
    
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    RearRight.setPower(0);
    RearLeft.setPower(0);

    


  }
}




