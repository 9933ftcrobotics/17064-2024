package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Dry Cycle CC", group = "")
//@Disabled
public class Dry_Cycle_CC extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime ();
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
    int state,cycles;
    double TimeElapsed,Prev_Time;
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
            timer.reset();
            target = ArmConstants.armRest;
            ArmExtender.setTargetPosition(ArmConstants.outRest);
            while (opModeIsActive()) {
                TimeElapsed = timer.seconds()-Prev_Time;
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

                switch(state){
                    case 0:
                        Prev_Time = timer.seconds();
                        state = 10;
                        break;
                    case 10:
                        Prev_Time = timer.seconds();
                        target = ArmConstants.armRest;
                        ArmExtender.setTargetPosition(ArmConstants.outRest);
                        state = 20;
                        break;
                    case 20:
                        if(TimeElapsed > 5){state = 30;}
                        break;
                    case 30:
                        Prev_Time = timer.seconds();
                        target = ArmConstants.armHighBasket;
                        ArmExtender.setTargetPosition(ArmConstants.outHighBasket);
                        state = 40;
                        break;
                    case 40:
                        if(TimeElapsed > 5){state = 50;}
                        break;
                    case 50:
                        ArmExtender.setTargetPosition(ArmConstants.outRest);
                        state = 60;
                        break;
                    case 60:
                        if(ArmExtender.getCurrentPosition() < ArmConstants.outInEnough){state = 10;}
                        break;
                }

                telemetry.addData("Time", timer.seconds());
                telemetry.addData("Time Elapsed", TimeElapsed);
                telemetry.addData("State", state);

                telemetry.addData("Robot Angle", Robot_Angle);
                telemetry.addData("ArmExtender", ArmExtender.getTargetPosition());

                telemetry.update();
            }
        }
    }
}