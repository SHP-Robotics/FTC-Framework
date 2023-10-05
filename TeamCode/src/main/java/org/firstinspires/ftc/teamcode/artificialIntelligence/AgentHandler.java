package org.firstinspires.ftc.teamcode.artificialIntelligence;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.debug.Constants;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

import ai.onnxruntime.OnnxJavaType;
import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;

public class AgentHandler {
    OrtEnvironment env;
    OrtSession session;
    private static final String VUFORIA_KEY = "ATZ/F4X/////AAABmSjnLIM91kSRo8TfH6CpvkpQb02HUOXzsAmc9sWr5aQKwBP0+GpVCddkSd7qVIgzYGRsutM1OEr4dRHyoy7G3gE8kovM+mnw5nVVkEJQEOhXlUt8ZN23VxVEMHO9qDIcH4vEv6w105kXo9FLJlikfRmKzVjMF/YAS4bU9UQVYpVzXCrEaoSE67McYRahSc3JfFmVkMqUCS2DDqyBC3MkN/YsO+EPmjz4iDIGz9HkSHkxylCOQ3rSHZQwZoGyrPJfkpl4XJoH+dKIawL3KeEWbMOIwDFR/IECVa8SNEeeaThDF3pvha2lTtdtgh5XLIcdSi27UQVTnaaM+5/G2gHLPMQ4n3DHIg4CQvmChLZTwD65";

    private VuforiaLocalizer vuforia;

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

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the frame queue capacity (3 has always worked for us)

        vuforia.setFrameQueueCapacity(3);
    }

    public ByteBuffer getFrame() {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();

        for (int i = 0; i < frame.getNumImages(); i++) {
            Image image = frame.getImage(i);

            if (image.getFormat() == PIXEL_FORMAT.GRAYSCALE) {
                int width = image.getWidth();

                int height = image.getHeight();

                // Once you have selected a specific RGB565 image, you can fill a ByteBuffer with its

                // pixel contents.

                ByteBuffer buf = image.getPixels();

                // And you can fill an array of bytes with the contents of the byte buffer (two bytes per pixel)

                byte[] bytes = new byte[2 * width * height];

                return buf;
            }
        }

        return null;
    }

    public float[] getOffset(ByteBuffer realPositionCamera1, ByteBuffer realPositionCamera2, ByteBuffer offsetPositionCamera1, ByteBuffer offsetPositionCamera2) {
        try {
            Map<String, OnnxTensor> tensor = new HashMap<String, OnnxTensor>() {{
                put("realPositionCamera1", OnnxTensor.createTensor(env, realPositionCamera1, new long[]{1, 1}, OnnxJavaType.FLOAT));
                put("realPositionCamera2", OnnxTensor.createTensor(env, realPositionCamera2, new long[]{1, 1}, OnnxJavaType.FLOAT));
                put("offsetPositionCamera1", OnnxTensor.createTensor(env, offsetPositionCamera1, new long[]{1, 1}, OnnxJavaType.FLOAT));
                put("offsetPositionCamera2", OnnxTensor.createTensor(env, offsetPositionCamera2, new long[]{1, 1}, OnnxJavaType.FLOAT));
            }};

            float[] output = new float[3];

            // Run the model
            OrtSession.Result result = session.run(tensor);

            output[0] = ((float[])result.get(0).getValue())[0] * Constants.UNIT_LENGTH;
            output[1] = ((float[])result.get(1).getValue())[0] * Constants.UNIT_LENGTH;
            output[2] = ((float[])result.get(2).getValue())[0] * Constants.UNIT_ROTATION;

            return output;

        } catch (OrtException e) {
            throw new RuntimeException(e);
        }
    }
}