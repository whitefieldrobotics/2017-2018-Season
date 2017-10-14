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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="JewlRed", group="Linear Opmode")
@Disabled
public class JewlRed extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor  leftDrive1  = null;
    public DcMotor  rightDrive1  = null;
    public DcMotor  leftDrive2  = null;
    public DcMotor  rightDrive2  = null;
    Servo arm = null;
    ColorSensor colorSensor;    // Hardware Device Object


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        arm = hardwareMap.get(Servo.class, "arm");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);




        // Wait for the game to start (driver presses PLAY)
        colorSensor.enableLed(true);

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            arm.setPosition(.75);
            sleep(1500);
            telemetry.addData("Current POS: ", arm.getPosition());
            telemetry.update();
            if (colorSensor.red()>colorSensor.blue()){
                telemetry.addData("Ball Color is: ", "Red");
                telemetry.update();

                leftDrive1.setPower(-.8);
                leftDrive2.setPower(-.8);

                rightDrive1.setPower(.8);
                rightDrive1.setPower(.8);
                sleep(700);

            }
            else {
                telemetry.addData("Ball Color is: ", "Blue");
                telemetry.update();
                leftDrive1.setPower(.8);
                leftDrive2.setPower(.8);

                rightDrive1.setPower(-.8);
                rightDrive1.setPower(-.8);
                sleep(700);
            }
            leftDrive1.setPower(0);
            leftDrive2.setPower(0);

            rightDrive1.setPower(0);
            rightDrive1.setPower(0);

            sleep(1000);
            arm.setPosition(0.0);
            sleep(1000);
            break;


        }
    }
}
