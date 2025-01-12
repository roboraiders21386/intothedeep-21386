/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Autonomous  for only vision detection using OpenCV VisionPortal and park
 */
@Autonomous(name = "Auton_left Meet 2", group = "00-Autonomous", preselectTeleOp = "TeleOpPS5")
public class AUTON_LEFT_OLDSERVO extends LinearOpMode {

    private Servo specimen;
    private DcMotor Lift;
    private DcMotor Lift2;
    private Servo Rotation;
    private Servo sample;
    private Servo Wrist;
    private int SPECIMEN_LIFT = 2000;
    private double OPEN_SPECIMEN_CLAW = 0.5;
    private double CLOSE_SPECIMEN_CLAW = 0.77;
    private double liftPow = 0.875;


    @Override
    public void runOpMode() throws InterruptedException {


        //extension initialization
        sample = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        Wrist = hardwareMap.get(Servo.class, "wrist");

        //Lift: Initialize lift
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotor.Direction.REVERSE);
        //Lift2: Initialize
        Lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotor.Direction.FORWARD);

        sample.setPosition(0);

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        //For Robot-centric ts
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenDropPose = new Pose2d(0,0,0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d pickSamplePose = new Pose2d(0,0,0);
        Pose2d pickSpecimen = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        //initPose -> midwayPose 1 -> specimenDropPose -> pickSamplePose --> sampleDropPose --> midwayPose2 --> parkPose
        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        midwayPose1 = new Pose2d(14, 40, Math.toRadians(0));
        midwayPose2 = new Pose2d(10, -12, Math.toRadians(0));
        pickSamplePose = new Pose2d(16.5,38, Math.toRadians(0)); //TODO: Do splineToConstantHeading
        Pose2d sampleDropPose = new Pose2d(0, 45.5, Math.toRadians(-45));
        Pose2d sampleDropPoseFloor = new Pose2d(0, 47, Math.toRadians(-45));
        Pose2d pickSamplePose2 = new Pose2d(16,51, Math.toRadians(0));
        Pose2d pickSamplePose3 = new Pose2d(23,57.5, Math.toRadians(20));
        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(60, 30, Math.toRadians(-90));//changed from 90 to 0 to face forward
        Pose2d parkPose2 = new Pose2d(60, 20, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, initPose);

        //Start with everything set
        sample.setPosition(0);
        Wrist.setPosition(0);
        Rotation.setPosition(0.1644-0.025);
        goBackHome();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());
        //Move robot to midwayPose1
        raiseLift();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(sampleDropPose, -Math.sqrt(2)/2)
                        .build());
        scoreHighBasket();
        safeWaitSeconds(0.25);
        goBackHome();
        //Move robot to pick up a sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(pickSamplePose, 0)
                        .build());

        // TODO: pick up a sample
        Rotation.setPosition(0.34-0.025);
        safeWaitSeconds(1.25);
        sample.setPosition(0);
        safeWaitSeconds(0.25);
        Rotation.setPosition(0.3-0.025);
        safeWaitSeconds(0.5);
        raiseLift();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleDropPoseFloor.position, sampleDropPose.heading)
                        .build());
        scoreHighBasket();
        safeWaitSeconds(0.25);
        goBackHome();
        //Move robot to pick up a sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(pickSamplePose2, 0)
                        .build());

        // TODO: pick up a sample
        Rotation.setPosition(0.34-0.025);
        safeWaitSeconds(1.25);
        sample.setPosition(0);
        safeWaitSeconds(0.25);
        Rotation.setPosition(0.3-0.025);
        safeWaitSeconds(0.5);
        raiseLift();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleDropPoseFloor.position, sampleDropPose.heading)
                        .build());
        scoreHighBasket();
        safeWaitSeconds(1);
        goBackHome();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(pickSamplePose3, Math.tan(Math.toRadians(20)))
                        .build());

        // TODO: pick up a sample
        Rotation.setPosition(0.34-0.025);
        safeWaitSeconds(1.25);
        sample.setPosition(0);
        safeWaitSeconds(0.25);
        Rotation.setPosition(0.3-0.025);
        safeWaitSeconds(0.5);
        raiseLift();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleDropPoseFloor.position, sampleDropPose.heading)
                        .build());
        scoreHighBasket();
        safeWaitSeconds(0.25);
        goBackHome();
        Rotation.setPosition(0.2-0.025);
        //Move robot to park in Observation Zone
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //TODO: after samples go here
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose2.position, parkPose2.heading)
                        .build());


    } // runAutonoumousMode

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
    public void goBackHome(){
        //Bring the lift down
        Wrist.setPosition(0);
        Rotation.setPosition(0.165-0.025);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(0);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(-1 * liftPow);
        //while (Lift.isBusy()) {
        telemetry.addData("Current Position", Lift.getCurrentPosition());
        telemetry.addData("Target Position", Lift.getTargetPosition());
        telemetry.addData("Current Position", Lift2.getCurrentPosition());
        telemetry.addData("Target Position", Lift2.getTargetPosition());
        telemetry.update();
        //}
    }
    public void raiseLift() {
        Wrist.setPosition(0);
        Rotation.setPosition(0.165-0.025);
        Lift.setTargetPosition(3080);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
        Lift2.setTargetPosition(3080);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(-1 * liftPow);
    }
    public void scoreHighBasket(){
        Rotation.setPosition(0.1-0.025);
        safeWaitSeconds(0.5);
        Wrist.setPosition(1);
        safeWaitSeconds(0.25);
        sample.setPosition(0.3);
        //while (Lift.isBusy()) {
        telemetry.addData("Current Position", Lift.getCurrentPosition());
        telemetry.addData("Target Position", Lift.getTargetPosition());
        telemetry.addData("Current Position", Lift2.getCurrentPosition());
        telemetry.addData("Target Position", Lift2.getTargetPosition());
        telemetry.update();
    }

}