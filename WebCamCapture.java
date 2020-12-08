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
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * This class will capture and return images from a webcam plugged into the Control Hub
 * USB port. Cannot be used at the same time as Vuforia. Use VuforiaCapture class if using
 * Vuforia in your OpMode. Does not support phone cameras. To monitor phone camera use the
 * VuforiaCapture class.
 */
public class WebCamCapture
{
    private static final String TAG = "WebCamCapture";

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    private CameraManager           cameraManager;
    //private WebcamName              cameraName;
    private CameraName              cameraName;
    private Camera                  camera;
    private CameraCaptureSession    cameraCaptureSession;

    private Size size;
    private int fps;
    private String  errorMessage;

    /** The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded. */
    private EvictingBlockingQueue<Bitmap> frameQueue;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    /**
     * Constructor.
     * @param map HardwareMap from calling opMode.
     * @param cameraName Name of webcam from hardware configuration.
     */
    public WebCamCapture(@NonNull HardwareMap map, @NonNull String cameraName)
    {
        this.cameraName = map.get(WebcamName.class, cameraName);

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();

        initializeFrameQueue(10);

        openCamera();

        startCamera();
    }

    /**
     * Get an image from the webcam capture queue.
     * @return Bitmap image.
     */
    public Bitmap getImage()
    {
        if (cameraCaptureSession != null)
            return frameQueue.poll();
        else
            return null;
    }

    /**
     * Return camera max frames per second.
     * @return Camera FPS.
     */
    public int getFPS()
    {
        return fps;
    }

    /**
     * Return the image size returned by camera.
     * @return Image Size
     */
    public Size getImageSize()
    {
        return size;
    }

    private void initializeFrameQueue(int capacity)
    {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly. This avoids a buildup of frames in memory */

        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<Bitmap>(capacity));

        frameQueue.setEvictAction(new Consumer<Bitmap>()
        {
            @Override public void accept(Bitmap frame)
            {
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera()
    {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);

        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, Continuation.create(callbackHandler, new Camera.StateCallback()
        {
            @Override
            public void onOpened(@NonNull Camera camera)
            {
                RobotLog.v(TAG, "camera opened");
            }

            @Override
            public void onOpenFailed(@NonNull CameraName cameraName, @NonNull Camera.OpenFailure reason)
            {
                errorMessage = String.format( "camera open failed: %s (%s)", cameraName, reason);
                RobotLog.e(TAG, errorMessage);
            }

            @Override
            public void onClosed(@NonNull Camera camera)
            {
                RobotLog.v(TAG, "camera closed");
            }

            @Override
            public void onError(@NonNull Camera camera, Camera.Error cameraError)
            {
                errorMessage = String.format( "camera error: %s (%s)", cameraName, cameraError);
                RobotLog.e(TAG, errorMessage);
            }
        }));

        if (camera == null)
            throw new RuntimeException(errorMessage);
    }

    private void startCamera()
    {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();

        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat))
            throw new RuntimeException("image format not supported");

        size = cameraCharacteristics.getDefaultSize(imageFormat);
        fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();

        try
        {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault()
            {
                @Override public void onConfigured(@NonNull CameraCaptureSession session)
                {
                    try
                    {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);

                        session.startCapture(captureRequest,
                                             new CameraCaptureSession.CaptureCallback()
                                             {
                                                 @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame)
                                                 {
                                                     /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                                      * for the duration of the callback. So we copy here manually. */
                                                     Bitmap bmp = captureRequest.createEmptyBitmap();
                                                     cameraFrame.copyToBitmap(bmp);
                                                     frameQueue.offer(bmp);
                                                 }
                                             },

                                             Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback()
                                             {
                                                 @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber)
                                                 {
                                                     //RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                                 }
                                             })
                                            );

                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();

        if (cameraCaptureSession ==  null)
            throw new RuntimeException("failed to create webcam capture session");
        else
            RobotLog.v(TAG, "Webcam capture session started");
    }

    private void stopCamera()
    {
        if (cameraCaptureSession != null)
        {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    /**
     * Stop WebCam image capture and close the camera.
     */
    public void closeCamera()
    {
        stopCamera();

        if (camera != null)
        {
            camera.close();
            camera = null;
        }
    }

    private boolean contains(int[] array, int value)
    {
        for (int i : array)
        {
            if (i == value) return true;
        }

        return false;
    }

    /**
     * Returns a CameraName object for phone internal camera. Phone camera not currently supported
     * by the FTC SDK so this is hidden for now. If phone is supported via the camera classes, this
     * would be part of genericzing this class to support both camera types. In theory, the method
     * that opens the camera should take either USB or phone camera and the rest of the code would
     * work with either camera. Right now, trying to open the camera with built-in camera returns a
     * "not supported" error.
     * @param camera Camera "direction", Front or Back.
     * @return CameraName object to use in opening camera for capture.
     */
    private CameraName getBuiltInCameraName(VuforiaLocalizer.CameraDirection camera)
    {
        return ClassFactory.getInstance().getCameraManager().nameFromCameraDirection(camera);
    }
}
