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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;




@Autonomous(name="BlueTeam", group ="Autonomous")
@Disabled
public class BlueTeam extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    String key = null;



    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor  leftDrive1  = null;
    public DcMotor  rightDrive1  = null;
    public DcMotor  leftDrive2  = null;
    public DcMotor  rightDrive2  = null;
    Servo arm = null;
    ColorSensor colorSensor;    // Hardware Device Object

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "ARXYTA3/////AAAAGUG5gdoOFUSSrZl2BfIbZgwCV+73vgiMY1Gdkqe9zPzLWISzJqaqxH1i2POM/CUX5F5P14W7Jw+yKadO/DCxys5xJPm0PmNI7bhaRFv2pF8HhniR9jjWeOoeMQ6CNHp/KwZnjo+2rxQniZSavk0U6HSieJyXgDvzhL7ClrLXcVoQj5j2UEnkGeBswKd5shky5nfM1//QkZkqvp0GoNQoKR9z4Wp+7Ev0sTwipEoTE3gMI5VySCe4Py8TBRsjohDT26BbkAmQjC+nwB91Pg6DYYQyI7sh15eENtaGD+DwHMs2J8fpW+NdydKi30p7IL0ia29QfstNlS47FGQ6ZOhZJjJpk2y113NIMDmJhqvrn5R7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        arm = hardwareMap.get(Servo.class, "arm");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);



        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            int heading = modernRoboticsI2cGyro.getHeading();

            for (int i = 0; i < 60; i++) {

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        key = "LEFT";
                    } else {
                        if (vuMark == RelicRecoveryVuMark.CENTER) {
                            key = "CENTER";
                        } else {
                            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                key = "RIGHT";
                            }
                        }
                    }
                }
                telemetry.addData("variable i : ", i);
                telemetry.update();
                sleep(50);
            }
            if (key == null) {
                telemetry.addData("no target found", key);
                telemetry.update();
                sleep(3000);
                break;
            } else {
                for (int i = 0; i < 60; i++) {
                    sleep(50);
                    telemetry.addData("putting gliph into the ", key, " key");
                    telemetry.update();
                }



                arm.setPosition(.75);
                sleep(1500);
                telemetry.addData("Current POS: ", arm.getPosition());
                telemetry.update();
                if (colorSensor.red()>colorSensor.blue()){
                    telemetry.addData("Ball Color is: ", "Red");
                    telemetry.update();

                    leftDrive1.setPower(.8);
                    leftDrive2.setPower(.8);

                    rightDrive1.setPower(-.8);
                    rightDrive1.setPower(-.8);
                    sleep(700);

                }
                else {
                    telemetry.addData("Ball Color is: ", "Blue");
                    telemetry.update();
                    leftDrive1.setPower(-.8);
                    leftDrive2.setPower(-.8);

                    rightDrive1.setPower(.8);
                    rightDrive1.setPower(.8);
                    sleep(700);
                }
                leftDrive1.setPower(0);
                leftDrive2.setPower(0);

                rightDrive1.setPower(0);
                rightDrive1.setPower(0);

                sleep(1000);
                arm.setPosition(0.0);
                sleep(1000);





            }

            break;
        }
    }
}
