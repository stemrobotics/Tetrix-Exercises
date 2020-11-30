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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The VuForia function
 * is  packaged as utility class within the main opMmode class (inner class). The findVuMark class
 * is generic usable for any single VuMark. It could be moved out of this example to a separate
 * class or a library class.
 *
 * This version uses a USB webcam connected to a Control Hub.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}. Note: there is a different license key for
 * use with a webcam as opposed to the phone camera.
 *
 *  VuMark is like a bar code. It is an image that contains encoded variable information. For the
 *  Relic Recovery game, the VuMark is the image of a temple. Encoded on that image in hexagonal
 *  dots is a code indicating left, center and right. Vuforia is used to locate the image in the
 *  camera field of view and extract the code returning that to your program. FIRST included a
 *  custom enum class to display the code (also called an instance id) as text.
 */

@Autonomous(name="VuMark Id - Webcam", group ="Exercises")
@Disabled
public class VuMarkIdentificationWebcam extends LinearOpMode
{
    VuMarkFinder        vmf;
    RelicRecoveryVuMark vuMark;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Retrieve the camera we are to use from robot configuration.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create an instance of VuMarkFinder. This can take some time to complete.
        vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", true, webcamName);

        // Start VuForia background process looking for vumarks in camera field of view.
        // We start this before waitForStart so that the camera feed can be viewed on the
        // driver station during the INIT wait. During init wait click the 3 dot menu on DS
        // to see the camera feed. Only available during init wait and you have to tap the
        // preview area to update the image (not a constantly updating feed).

        vmf.activate();

        telemetry.addData("Mode", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // See if a vumark is visible.
            if (vmf.findVuMark())
            {
                // Convert vumark instance  id to game specific id.
                vuMark = RelicRecoveryVuMark.from(vmf.instanceId);

                telemetry.addData("VuMark", "%s visible", vuMark);

                //telemetry.addData("Pose", vmf.formatPose(vmf.pose));

                telemetry.addData("X Y Z", "X=%f  Y=%f  Z=%f", vmf.tX, vmf.tY, vmf.tZ);
            }
            else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();

            idle();
        }
    }

    /**
     * VuForia VuMark finder class.
     */
    public class VuMarkFinder
    {
        private VuforiaLocalizer    vuforia;
        private VuforiaTrackables   trackables;
        private VuforiaTrackable    template;

        public VuMarkInstanceId     instanceId;
        public OpenGLMatrix         pose;
        public double               tX, tY, tZ, rX, rY, rZ;

        /** Constructor.
         * Create an instance of the class.
         * @param hMap HardwareMap object.
         * @param assetName Name of the asset file containing the VuMark definition.
         * @param includeViewer True to display camera viewer on RC phone.
         * @param cameraName Webcam name from robot configuration.
         */
        public VuMarkFinder(HardwareMap hMap,
                            String assetName,
                            boolean includeViewer,
                            WebcamName cameraName)
        {
            VuforiaLocalizer.Parameters parameters;

            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
             */

            if (includeViewer)
            {
                int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            }
            else
                // OR...  Do Not Activate the Camera Monitor View, to save power
                parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = "AaDAvEH/////AAABmT7XsefufE2DuYYnYCxrBsQQ9FrK/39uullPQn7b/XVUOAU9eFLRcRYYm1JY0ChQpml/x1CPv5kyBtc5rwVrTM0I2/VcBKiulYWzGM8kZDYAIwIwpncnYbyxCHgN80KAZplNqMiL0lWP1SKFE1jXojLSu33a+gcyDvRQCJtHteF976mcXTsadxZCJFhUGx198hOmuK5HNTwjvNoxcEUmF5BOS9hLDBCrZnfTHYbYyKoKMX17a3K7FR+T8C8s+zOGvKXc9vtjNTJDUZ0D1gvyDSlzu52fHXAVTb7HRJN9rapGZ6wyqn2UGay5dxjCOFsxsWZFSzPRN3zZri//WalFQysr6MWUEykjDJhGKZucvBHr";

            /*
             * We also indicate which camera on the RC that we wish to use.
             * Here we chose the back (HiRes) camera (for greater range), but
             * for a competition robot, the front camera might be more convenient.
             */
            //parameters.cameraDirection = camera;
            parameters.cameraName = cameraName;
            parameters.useExtendedTracking = false;

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            /*
             * Load the data set containing the VuMark. This code supports 1 VuMark.
             */
            trackables = vuforia.loadTrackablesFromAsset(assetName);
            template = trackables.get(0);
            template.setName(assetName); // can help in debugging; otherwise not necessary
        }

        /**
         * Activate VuForia image processing. Call after waitForStart().
         */
        public void activate()
        {
            trackables.activate();
        }

        /**
         * Call to find out if VuMark is visible to the phone camera.
         * @return True if VuMark found, false if not.
         */
        public boolean findVuMark()
        {
            // See if any of the instances of the template are currently visible.
            instanceId = ((VuforiaTrackableDefaultListener) template.getListener()).getVuMarkInstanceId();

            if (instanceId != null)
            {
                pose = ((VuforiaTrackableDefaultListener) template.getListener()).getFtcCameraFromTarget();

                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }

                return true;
            }
            else
            {
                pose = null;
                return false;
            }
        }

        /**
         * Format pose object for human viewing.
         * @param pose Pose object returned when VuMark is found.
         * @return Pose description.
         */
        String formatPose(OpenGLMatrix pose)
        {
            return (pose != null) ? pose.formatAsTransform() : "null";
        }
    }
}

