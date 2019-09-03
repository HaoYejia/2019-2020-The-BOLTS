/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumOpMode", group="Iterative Opmode")
//@Disabled
public class BasicOpMode2_Iterative extends OpMode
{
    MecanumWheel mechanumW = new MecanumWheel();  //麦轮控制载入
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFtWheel = null;
    private DcMotor leftBdWheel = null;
    private DcMotor rightFtWheel = null;
    private DcMotor rightBdWheel = null;
    private DcMotor lift = null;
    private CRServo servo1 = null;  //舵机初始化
    private CRServo servo2 = null;
    private CRServo collect = null;
    final double servo_power = 0.2;
    final double collect_power = 1.0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        mechanumW.init();

        lift = hardwareMap.dcMotor.get("lift");
        leftFtWheel = hardwareMap.dcMotor.get("leftFtWheel");
        leftBdWheel = hardwareMap.dcMotor.get("leftBdWheel");
        rightFtWheel = hardwareMap.dcMotor.get("rightFtWheel");
        rightBdWheel = hardwareMap.dcMotor.get("rightBdWheel");
        servo1 = hardwareMap.crservo.get("leftServo");
        servo2 = hardwareMap.crservo.get("rightServo");
        collect = hardwareMap.crservo.get("collect");

        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
        collect.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFtWheel.setPower(0);
        rightFtWheel.setPower(0);
        leftBdWheel.setPower(0);
        rightBdWheel.setPower(0);
    }

    public void setMecNumMotorPower(){
        leftFtWheel.setPower(0.8 * mechanumW.fl_value);
        rightFtWheel.setPower(0.8 * -mechanumW.fr_value);
        leftBdWheel.setPower(0.8 * mechanumW.rl_value);
        rightBdWheel.setPower(0.8 * mechanumW.rr_value);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        mechanumW.setMode(gamepad1.left_stick_y,gamepad1.left_stick_x,-gamepad1.right_stick_x);
        mechanumW.calculatePower();
        setMecNumMotorPower();
        // 根据摇杆动作，映射到麦轮驱动中的动作指令，进而控制机器人底盘运动

        if (gamepad1.dpad_up){
            servo1.setPower(servo_power);
            servo2.setPower(-servo_power);
        }else if (gamepad1.dpad_down){
            servo1.setPower(-servo_power);
            servo2.setPower(servo_power);
        }else{
            servo1.setPower(0);
            servo2.setPower(0);
        }


        /**
         * ↓收集爪控制器
         */
        if (gamepad1.left_bumper){
            collect.setPower(collect_power);
        }else if (gamepad1.right_bumper){
            collect.setPower(-collect_power);
        }else{
            collect.setPower(0);
        }

        /**
         * ↓控制器
         */
        if (gamepad1.x){
           lift.setPower(0.9);
        }else if (gamepad1.a){
            lift.setPower(-0.5);
        }else {
            lift.setPower(0);
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor", "rightFtWheel: " + rightFtWheel.getPower());

    }

    @Override
    public void stop() {
    }

}
