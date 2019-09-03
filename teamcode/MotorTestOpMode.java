package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="MotorTestOpMode", group="TestOpmode")
@Disabled
public class MotorTestOpMode extends LinearOpMode {

    private DcMotor motorTest;
    double power = 0.5;

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.dcMotor.get("motorTest"); //电机的名称：motorTest
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        motorTest.setPower(power);
        telemetry.addData("The power:",power);

        sleep(2000); //让电机持续转动2秒，在这之间不做任何动作

        power = 0;
        motorTest.setPower(power); //将power重设为零，停下电机
        telemetry.addData("The power:", power);
        }
    }