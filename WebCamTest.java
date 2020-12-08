
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        webCam = new WebCamCapture(hardwareMap, "Webcam 1");

        server = new MJpegServer();

        // Most cameras run at 30 fps but setting the server to run faster smooths
        // out the video feed somewhat.
        server.setFPS(50);

        telemetry.addData("Mode", "Press Play to start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "Streaming started fps=" + webCam.getFPS());
        telemetry.update();

        while (opModeIsActive())
        {
            cameraImage = webCam.getImage();

            server.setImage(cameraImage);

            idle();
        }

        server.stop();
        webCam.closeCamera();
    }
}
