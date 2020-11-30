/* Copyright (c) 2020 FIRST. All rights reserved.
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

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This OpMode illustrates how to use the WebCamCapture class to get images from
 * a webcam plugged into the Control Hub USB port and stream those images over
 * the network connection to a web browser using MJpegServer class. Use this URL
 * in the browser:
 * http://192.168.43.1:8888
 */
@TeleOp(name="WebCamTest", group ="Exercises")
//@Disabled
public class WebCamTest extends LinearOpMode
{

    WebCamCapture       webCam;
    MJpegServer         server;
    Bitmap              cameraImage;

    @Override
    public void runOpMode() throws InterruptedException
    {
        webCam = new WebCamCapture(hardwareMap.get(WebcamName.class, "Webcam 1"));

        server = new MJpegServer();

        // Most cameras run at 30 fps but setting the server to run faster smoothes
        // out the vide feed sommewhat.
        server.setFPS(50);

        telemetry.addData("Mode", "Press Play to start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "Streaming started fps=" + webCam.getFPS());
        telemetry.update();

        while (opModeIsActive())
        {
            cameraImage = webCam.getImage();

            server.setBitmap(cameraImage);

            idle();
        }

        server.stop();
        webCam.closeCamera();
    }
}
