package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@Config
@TeleOp(name = "Arm_Tuner_CC", group = "")
//@Disabled
public class ArmTuner_CC extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
double power;
    public static int target = 0;

    private final double ticks_in_degree = 2786.2 / 360.0;



    private DcMotor ArmExtender;
    private DcMotor arm;


    @Override
    public void runOpMode() {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        arm = hardwareMap.get(DcMotor.class, "ArmLift");
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");

        ArmExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtender.setTargetPosition(0);
        ArmExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmExtender.setDirection(DcMotor.Direction.REVERSE);


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {

            ArmExtender.setPower(0.8);
            while (opModeIsActive()) {
                controller.setPID(p,i,d);
                int armpos = arm.getCurrentPosition();

                double pid = controller.calculate(armpos,target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;
                power = pid + ff;
                power = MathUtils.clamp(power,-0.4,1);
                arm.setPower(power);


                telemetry.addData("ArmLift", arm.getCurrentPosition());
                telemetry.addData("ArmLift Target", target);
                telemetry.addData("ArmExtender", ArmExtender.getCurrentPosition());


                telemetry.update();
            }
        }
    }
}