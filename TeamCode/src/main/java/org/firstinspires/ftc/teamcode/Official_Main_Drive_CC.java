package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Official_Main_Drive_CC", group = "")
//@Disabled
public class Official_Main_Drive_CC extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime ();
    private DcMotor ArmExtender;
    private DcMotor arm;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor RearLeft;
    private DcMotor RightClimb;
    private DcMotor LeftClimb;
    private CRServo InTake;
//    private DcMotor Arm;
//    private Servo Claw;

  /*private DcMotor RightMotor;
  private DcMotor LeftMotor;*/

    private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    boolean Overidden = false;
    double SpeedReducer = 0;

    private PIDController controller;
    public static double p = 0.01, i = 0.05, d = 0.00075;
    public static double f = 0.5;

    public static int target = 0;

    private final double ticks_in_degree = 2786.2 / 360.0;


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
    double power;
    int Count = 0;

    @Override
    public void runOpMode() {
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        arm = hardwareMap.get(DcMotor.class, "ArmLift");
        FrontRight = hardwareMap.dcMotor.get("RightFront");
        FrontLeft = hardwareMap.dcMotor.get("LeftFront");
        RearRight = hardwareMap.dcMotor.get("RearRight");
        RearLeft = hardwareMap.dcMotor.get("RearLeft");
        RightClimb = hardwareMap.dcMotor.get("RightHang");
        LeftClimb = hardwareMap.dcMotor.get("LeftHang");
        InTake = hardwareMap.get(CRServo.class, "InTake");
        ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtender.setTargetPosition(0);
        ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        RightClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightClimb.setTargetPosition(0);
        RightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftClimb.setTargetPosition(0);
        LeftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmExtender.setDirection(DcMotor.Direction.REVERSE);
        LeftClimb.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        temp = true;
        count = 0;

        controller = new PIDController(p,i,d);

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
            RightClimb.setPower(1);
            LeftClimb.setPower(1);
            ArmExtender.setPower(0.65);
            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armpos = arm.getCurrentPosition();

                double pid = controller.calculate(armpos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                power = pid + ff;
                power = MathUtils.clamp(power,-0.4,1);
                arm.setPower(power);
                telemetry.addData("ArmLift Current", arm.getCurrentPosition());
                telemetry.addData("ArmLift Target", target);
                telemetry.addData("ArmExtender", ArmExtender.getCurrentPosition());


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


                if (!Overidden) {
                    if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.a || gamepad1.y){
                        Overidden = true;
                    }


                    else if (gamepad2.dpad_down) {
                        ArmExtender.setTargetPosition(ArmConstants.outGround);
                        target = ArmConstants.armGround;
                        InTake.setPower(-1);
                    } else if (gamepad2.dpad_right) {
                        target = ArmConstants.armBasket;
                        ArmExtender.setTargetPosition(ArmConstants.outLowBasket);
                } else if (gamepad2.dpad_up) {

                    if (gamepad2.y) {
                        target = ArmConstants.armAboveHighBasket;
                    } else {
                        target = ArmConstants.armHighBasket;
                        ArmExtender.setTargetPosition(ArmConstants.outHighBasket);

                    }
            } else if (gamepad2.a) {
                target = ArmConstants.armRest;
                ArmExtender.setTargetPosition(ArmConstants.outSub);
                InTake.setPower(-1);


            } else {
                ArmExtender.setTargetPosition(ArmConstants.outRest);
                if (ArmExtender.getCurrentPosition() < ArmConstants.outInEnough) {
                    target = ArmConstants.armRest;
                }

                InTake.setPower(0);
            }

        }
                else{
                    if(gamepad1.dpad_up){
                        target=target+5;

                    }
                    else if(gamepad1.dpad_down){

                        target=target-5;
                    }
                    if(gamepad1.y){
                        ArmExtender.setTargetPosition(ArmExtender.getTargetPosition()+25);
                    }
                    else if(gamepad1.a){
                        ArmExtender.setTargetPosition(ArmExtender.getTargetPosition()-25);
                    }
                }

                if(gamepad1.start){
                    ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Overidden = false;
                }

                if(gamepad1.back){
                    Overidden = false;
                }



                if (gamepad1.right_bumper){
                    InTake.setPower(1); }

                if (gamepad1.right_stick_button){
                    RightClimb.setTargetPosition(-2200);
                    LeftClimb.setTargetPosition(-2200);
                }
                else {

                    RightClimb.setTargetPosition(0);
                    LeftClimb.setTargetPosition(0);
                }

                telemetry.addData("Robot Angle", Robot_Angle);
                telemetry.addData("ArmExtender", ArmExtender.getTargetPosition());

                telemetry.update();
            }
        }
    }
}