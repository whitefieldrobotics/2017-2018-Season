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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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




@Autonomous(name="Pillar Testing", group ="Autonomous")
public class PillarTesting extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    String key = null;


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrive1 = null;
    public DcMotor rightDrive1 = null;
    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive2 = null;
    Servo arm = null;
    ColorSensor colorSensor;    // Hardware Device Object

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    /* IntegratingGyroscope gro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    */

    @Override
    public void runOpMode() {
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "ARXYTA3/////AAAAGUG5gdoOFUSSrZl2BfIbZgwCV+73vgiMY1Gdkqe9zPzLWISzJqaqxH1i2POM/CUX5F5P14W7Jw+yKadO/DCxys5xJPm0PmNI7bhaRFv2pF8HhniR9jjWeOoeMQ6CNHp/KwZnjo+2rxQniZSavk0U6HSieJyXgDvzhL7ClrLXcVoQj5j2UEnkGeBswKd5shky5nfM1//QkZkqvp0GoNQoKR9z4Wp+7Ev0sTwipEoTE3gMI5VySCe4Py8TBRsjohDT26BbkAmQjC+nwB91Pg6DYYQyI7sh15eENtaGD+DwHMs2J8fpW+NdydKi30p7IL0ia29QfstNlS47FGQ6ZOhZJjJpk2y113NIMDmJhqvrn5R7";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftDrive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
        arm = hardwareMap.get(Servo.class, "arm");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        int flag = 0;
/*
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();
*/
        waitForStart();

        relicTrackables.activate();

        idle();



        modernRoboticsI2cGyro.calibrate();
        int heading = modernRoboticsI2cGyro.getHeading();


        while (opModeIsActive()) {


            for (int t = 0; t < 60; t++) {

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        flag = 3;
                        //left pillar
                    } else {
                        if (vuMark == RelicRecoveryVuMark.CENTER) {
                            flag = 2;
                            //center pillar
                            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                                flag = 1;
                                //right pillar
                            }
                        }
                    }
                }
                sleep(50);
            }
            //test comment
            arm.setPosition(.85);
            sleep(1800);


            //finds Jewel then knocks off jewel, knocks off back jewel
            if (colorSensor.red()>colorSensor.blue()){
                telemetry.addData("Ball Color is: ", "Red");
                telemetry.update();

                setLeftPower(-.1);
                setRightPower(-.1);

                sleep(800);
                arm.setPosition(0.60);
                sleep(100);

            }//knocks off front jewel
            else {
                telemetry.addData("Ball Color is: ", "Blue");
                telemetry.update();
                setLeftPower(.1);
                setRightPower(.1);
                sleep(800);
                arm.setPosition(0.60);
                sleep(1000);
            }
            setLeftPower(0);
            setRightPower(0);

            idle();


            //first

            //uses method from earlier to determine what pillar it should stop at
            for (int i = 0; i< flag; i++) {
                arm.setPosition(0.60);

                while (colorSensor.red() <= 0) {
                    setLeftPower(.055);
                    setRightPower(.055);
                    idle();
                }



                setLeftPower(0);
                setRightPower(0);
                sleep(50);
                idle();

                arm.setPosition(0);
                sleep(1000);
                //brakes robot
                setLeftPower(-.2);
                setRightPower(-.2);
                sleep(10);

                setLeftPower(.18);
                setRightPower(.18);
                sleep(250);

                setLeftPower(0);
                setRightPower(0);
                sleep(50);


                idle();
            }
            //turns towards pillar
            setLeftPower(.5);
            setRightPower(-.5);

            sleep(1000);
            //test

            setLeftPower(0);
            setRightPower(0);
            sleep(50);

            //LINE OF CODE CHANGE IGNORE COMMENT


            //drives toward pillar
            setLeftPower(.2);
            setRightPower(.2);
            sleep(100);


            idle();
            break;
        }
    }

    //METHODS
    //shortens the drive1, drive2 thing
    public void setRightPower(double rightPower) {
        rightDrive1.setPower(rightPower);
        rightDrive2.setPower(rightPower);
    }

    public void setLeftPower(double leftPower) {
        leftDrive1.setPower(leftPower);
        leftDrive2.setPower(leftPower);
    }
    //gyro stuff
    public void gyroTurn(double leftPower, double rightpower, int angle)throws InterruptedException{
        int heading = modernRoboticsI2cGyro.getHeading();
        if (angle < 179){
            runWithEncoders(200, -200, .4, .4, 600, "small kick for gyro");
        }else{
            runWithEncoders(-200, 200, .4, .4, 600, "small kick for gyro");
        }
        while(heading < angle){
            heading = modernRoboticsI2cGyro.getHeading();
            leftDrive1.setPower(leftPower);
            leftDrive2.setPower(leftPower);
            rightDrive1.setPower(rightpower);
            rightDrive2.setPower(rightpower);
            sleep(100);
        }
    }

    //encoder stuff
    public void runWithEncoders(int leftDistance, int rightDistance, double leftPower, double rightPower, int duration, String task) throws InterruptedException {
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
        rightDrive2.setTargetPosition(rightDistance);
        leftDrive2.setTargetPosition(leftDistance);
        rightDrive1.setTargetPosition(rightDistance);
        leftDrive1.setTargetPosition(leftDistance);
        sleep(100);
        leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(100);
        rightDrive2.setTargetPosition(rightDistance);
        leftDrive2.setTargetPosition(leftDistance);
        rightDrive1.setTargetPosition(rightDistance);
        leftDrive1.setTargetPosition(leftDistance);
        sleep(duration);
        sleep(100);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
        setLeftPower(0);
        setRightPower(0);
        sleep(100);
    }
}

