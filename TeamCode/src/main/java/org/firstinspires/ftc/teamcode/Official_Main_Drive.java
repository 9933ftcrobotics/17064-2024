package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;

@TeleOp(name = "Official_Main_Drive", group = "")
@Disabled
public class Official_Main_Drive extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime ();
    private DcMotor ArmExtender;
    private DcMotor arm;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor RearLeft;
    private CRServo InTake;
//    private DcMotor Arm;
//    private Servo Claw;

  /*private DcMotor RightMotor;
  private DcMotor LeftMotor;*/

    private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    double SpeedReducer = 0;


    //declare motor speed variables
    double RF, LF, RR, LR;

    //declare joystick position variables
    double X1, Y1, X2, Y2;

    //operational constants
    double joyScale = 0.7;  //0.5;

    double motorMax = 0.7;  //0.6;
    double Left_Stick_Angle, Left_Stick_Ratio, Left_Stick_Magnitude;
    double Left_Stick_Y, Left_Stick_X;
    double Robot_Angle, Output_Angle;
    double LTrigger = 0;
    int Count = 0;

    @Override
    public void runOpMode() {
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        arm = hardwareMap.get(DcMotor.class, "ArmLift");
        FrontRight = hardwareMap.dcMotor.get("RightFront");
        FrontLeft = hardwareMap.dcMotor.get("LeftFront");
        RearRight = hardwareMap.dcMotor.get("RearRight");
        RearLeft = hardwareMap.dcMotor.get("RearLeft");
        InTake = hardwareMap.get(CRServo.class, "InTake");
        ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtender.setTargetPosition(0);
        ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmExtender.setDirection(DcMotor.Direction.REVERSE);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        temp = true;
        count = 0;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);


        waitForStart();
        if (opModeIsActive()) {
            arm.setPower(1);
            ArmExtender.setPower(0.8);
            while (opModeIsActive()) {

                telemetry.addData("ArmLift", arm.getCurrentPosition());
                telemetry.addData("ArmExtender", ArmExtender.getCurrentPosition());

                telemetry.addData("motor position", arm.getCurrentPosition());

                LF = 0;
                RF = 0;
                LR = 0;
                RR = 0;
                X1 = 0;
                Y1 = 0;

                motorMax = 1;
                Left_Stick_Y = -gamepad1.left_stick_y;
                Left_Stick_X = gamepad1.left_stick_x;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                Robot_Angle = angles.firstAngle * -1;
                if (Left_Stick_Y != 0 || Left_Stick_X != 0) {
                    Left_Stick_Ratio = Left_Stick_X / Left_Stick_Y;


                    //if left stick y greater than 0
                    if (Left_Stick_Y > 0) {
      /*it creates this ratio left stick x/ left stick y, then it calulates the angle
      this is the same thing for the false just add 180 to the angle*/
                        Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio));
                    } else {
                        Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio)) + 180;
                        if (Left_Stick_Angle > 180) {
                            Left_Stick_Angle -= 360;
                        }
                    }
                    //it calculates the power in which direction based on the x and y
                    Left_Stick_Magnitude = Math.sqrt(Math.pow(Left_Stick_Y, 2)
                            + Math.pow(Left_Stick_X, 2));

                    //output angle is the way the robot wil go based on the joystick angle - the current robot angle
                    //the lines after it are just implementing them
                    Output_Angle = Left_Stick_Angle - Robot_Angle;
                    if (Output_Angle > 180) {
                        Output_Angle -= 360;
                    }
                    if (Output_Angle < -180) {
                        Output_Angle += 360;
                    }
                    LTrigger = (1 - gamepad1.left_trigger);
                    LTrigger = Math.max(LTrigger, 0.2);

                    //this will set a value for the x and y axis of the motor
                    Y1 = Math.cos(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;
                    X1 = Math.sin(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;

                }
                X2 = gamepad1.right_stick_x * joyScale;

                // Forward/back movement
                LF += Y1;
                RF += Y1;
                LR += Y1;
                RR += Y1;

                //Side to side movement
                LF += X1;
                RF -= X1;
                LR -= X1;
                RR += X1;

                //Rotation Movement
                LF += X2;
                RF -= X2;
                LR += X2;
                RR -= X2;

                //Motor Speed

                //Clip motor power values to +/- motorMax
                LF = Math.max(-motorMax, Math.min(LF, motorMax));
                RF = Math.max(-motorMax, Math.min(RF, motorMax));
                LR = Math.max(-motorMax, Math.min(LR, motorMax));
                RR = Math.max(-motorMax, Math.min(RR, motorMax));


                //Send values to the motors
                if (gamepad1.left_trigger > gamepad2.left_trigger) {
                    LTrigger = (0.75 - gamepad1.left_trigger);
                    LTrigger = Math.max(LTrigger, 0.2);
                } else {
                    LTrigger = (0.75 - gamepad2.left_trigger);
                    LTrigger = Math.max(LTrigger, 0.2);
                }

                FrontLeft.setPower(LF * LTrigger);
                FrontRight.setPower(RF * LTrigger);
                RearLeft.setPower(LR * LTrigger);
                RearRight.setPower(RR * LTrigger);
                if (gamepad2.dpad_down) {
                    arm.setTargetPosition(0);
                    InTake.setPower(-1);
                } else if (gamepad2.dpad_right) {
                    arm.setTargetPosition(750);
                    ArmExtender.setTargetPosition(660);
                } else if (gamepad2.left_bumper) {
                    ArmExtender.setTargetPosition(1799);
                } else if (gamepad2.dpad_up) {
                    arm.setTargetPosition(800);
                    ArmExtender.setTargetPosition(1900);
                } else if (gamepad2.dpad_left) {
                    arm.setTargetPosition(1000);
                } else {
                    ArmExtender.setTargetPosition(0);
                    arm.setTargetPosition(210);
                    InTake.setPower(0);
                }
                if (gamepad1.right_bumper){
                    InTake.setPower(1); }

                telemetry.addData("Robot Angle", Robot_Angle);
                telemetry.addData("ArmExtender", ArmExtender.getTargetPosition());
                telemetry.update();
            }
        }
    }
}