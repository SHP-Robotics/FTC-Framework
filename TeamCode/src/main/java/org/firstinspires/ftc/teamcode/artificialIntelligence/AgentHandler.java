package org.firstinspires.ftc.teamcode.artificialIntelligence;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;

public class AgentHandler {
    OrtEnvironment env;
    OrtSession session;

    private static final String VUFORIA_KEY = "ATZ/F4X/////AAABmSjnLIM91kSRo8TfH6CpvkpQb02HUOXzsAmc9sWr5aQKwBP0+GpVCddkSd7qVIgzYGRsutM1OEr4dRHyoy7G3gE8kovM+mnw5nVVkEJQEOhXlUt8ZN23VxVEMHO9qDIcH4vEv6w105kXo9FLJlikfRmKzVjMF/YAS4bU9UQVYpVzXCrEaoSE67McYRahSc3JfFmVkMqUCS2DDqyBC3MkN/YsO+EPmjz4iDIGz9HkSHkxylCOQ3rSHZQwZoGyrPJfkpl4XJoH+dKIawL3KeEWbMOIwDFR/IECVa8SNEeeaThDF3pvha2lTtdtgh5XLIcdSi27UQVTnaaM+5/G2gHLPMQ4n3DHIg4CQvmChLZTwD65";

    private VuforiaLocalizer vuforiaCamera1;
    private VuforiaLocalizer vuforiaCamera2;
    private VuforiaLocalizer vuforiaCamera3;

    private ByteBuffer realCamera1;
    private ByteBuffer realCamera2;
    private ByteBuffer realCamera3;

    private int pleaseWorkIndex = -1;

    public void initEnvironment(String modelPath) {
        try {
            env = OrtEnvironment.getEnvironment();
            OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
            session = env.createSession(modelPath, opts);
        } catch ( OrtException e) {
            throw new RuntimeException(e);
        }
    }

    public void closeEnvironment() {
        try {
            session.close();
            env.close();
        } catch (OrtException e) {
            throw new RuntimeException(e);
        }
    }

    public void initVuforia(HardwareMap map) {
        // Usual code to initialize Vuforia

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = map.get(WebcamName.class, "Webcam 1");
        vuforiaCamera1 = ClassFactory.getInstance().createVuforia(parameters);

        parameters.cameraName = map.get(WebcamName.class, "Webcam 2");
        vuforiaCamera2 = ClassFactory.getInstance().createVuforia(parameters);

        parameters.cameraName = map.get(WebcamName.class, "Webcam 3");
        vuforiaCamera3 = ClassFactory.getInstance().createVuforia(parameters);

        // This code probs doesn't work
        vuforiaCamera1.setFrameQueueCapacity(3);
        vuforiaCamera2.setFrameQueueCapacity(3);
        vuforiaCamera3.setFrameQueueCapacity(3);
    }

    private void getCameraFrame(CameraIdentifier cameraIdentifier) {
        VuforiaLocalizer.CloseableFrame frame;
        if (cameraIdentifier == CameraIdentifier.REAL_CAMERA_ONE) {
            frame = vuforiaCamera1.getFrameQueue().poll();
        } else if (cameraIdentifier == CameraIdentifier.REAL_CAMERA_TWO) {
            frame = vuforiaCamera2.getFrameQueue().poll();
        } else if (cameraIdentifier == CameraIdentifier.REAL_CAMERA_THREE) {
            frame = vuforiaCamera3.getFrameQueue().poll();
        }

        Image image = frame.getImage(pleaseWorkIndex);
        if (image.getFormat() == PIXEL_FORMAT.GRAYSCALE) {
            realCamera1 = image.getPixels();
        } else {
            throw new Error("The pleaseWorkIndex didn't work!");
        }
    }

    public void getCurrentFrames() {
        VuforiaLocalizer.CloseableFrame frame1 = vuforiaCamera1.getFrameQueue().poll();
        VuforiaLocalizer.CloseableFrame frame2 = vuforiaCamera2.getFrameQueue().poll();
        VuforiaLocalizer.CloseableFrame frame3 = vuforiaCamera3.getFrameQueue().poll();

        if (pleaseWorkIndex == -1) {
            for (int i = 0; i < frame1.getNumImages(); i++) {
                Image image1 = frame1.getImage(i);

                if (image1.getFormat() == PIXEL_FORMAT.GRAYSCALE) {
                    pleaseWorkIndex = i;

                    int width = image1.getWidth();

                    int height = image1.getHeight();

                    // Once you have selected a specific RGB565 image, you can fill a ByteBuffer with its

                    // pixel contents.

                    ByteBuffer buf = image1.getPixels();

                    realCamera1 = buf;
                }
            }
        } else {

        }

        Image image2 = frame2.getImage(pleaseWorkIndex);

        if (image2.getFormat() == PIXEL_FORMAT.GRAYSCALE) {
            int width = image2.getWidth();

            int height = image2.getHeight();

            // Once you have selected a specific RGB565 image, you can fill a ByteBuffer with its

            // pixel contents.

            ByteBuffer buf = image2.getPixels();

            // And you can fill an array of bytes with the contents of the byte buffer (two bytes per pixel)

            byte[] bytes = new byte[2 * width * height];

            realCamera2 = bytes;
        } else {
            throw new Error("Your code sucks");
        }

        Image image3 = frame3.getImage(pleaseWorkIndex);

        if (image3.getFormat() == PIXEL_FORMAT.GRAYSCALE) {
            int width = image3.getWidth();

            int height = image3.getHeight();

            // Once you have selected a specific RGB565 image, you can fill a ByteBuffer with its

            // pixel contents.

            ByteBuffer buf = image3.getPixels();

            // And you can fill an array of bytes with the contents of the byte buffer (two bytes per pixel)

            byte[] bytes = new byte[2 * width * height];

            realCamera3 = bytes;
        } else {
            throw new Error("Your code sucks");
        }
    }

    public float[] forward() {
        try {
            Map<String, OnnxTensor> tensor = new HashMap<String, OnnxTensor>(){{
                put("realCamera1", OnnxTensor.createTensor(env, ByteBuffer.wrap(realCamera1), new long[]{1}));
                put("realCamera2", OnnxTensor.createTensor(env, ByteBuffer.wrap(realCamera2), new long[]{1}));
                put("realCamera3", OnnxTensor.createTensor(env, ByteBuffer.wrap(realCamera3), new long[]{1}));
                put("offsetCamera1", OnnxTensor.createTensor(env, offsetCamera1, new long[]{1}));
                put("offsetCamera2", OnnxTensor.createTensor(env, offsetCamera2, new long[]{1}));
                put("offsetCamera3", OnnxTensor.createTensor(env, offsetCamera3, new long[]{1}));
            }};

            float[] output = new float[3];

            OrtSession.Result result = session.run(tensor);

            output[0] = ((float[])result.get(0).getValue())[0];
            output[1] = -((float[])result.get(1).getValue())[0];
            output[2] = ((float[])result.get(2).getValue())[0];

            return output;

        } catch (OrtException e) {
            throw new RuntimeException(e);
        }
    }
}