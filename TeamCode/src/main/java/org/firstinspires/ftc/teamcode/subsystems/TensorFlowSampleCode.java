package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class TensorFlowSampleCode {
    /*
    //ask tensor flow for a lit of objects recongized in the current video image
    List<Recognition> updated Recognitions = myTfod.getUpdatedRecognitions();
    //go through list and understand what type of recognitions we have
    if(updatedRecognitions != null){
        for(Recognition recongition:updatedRecognitions){
            telemetry.addData(">", recognition.getLabel())
            if(recongition.getLabel().equals("Name of Detection")){
                int leftXBoundary = (int) recognition.getLeft();
                int rightXBoundary = (int) recognition.getRight();
                int goldMineral Center = (leftXBoundary+rightXBoundary)/2;
                //how far away(can be + or -) to tell left or right
                int offset
            }
        }
    }
    */
    TfodProcessor myTfodProcessor;
    TfodProcessor.Builder myTfodProcessorBuilder;
    public TensorFlowSampleCode(HardwareMap hardwareMap){
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        setCustomFeatures();
        myTfodProcessor = myTfodProcessorBuilder.build();
    }
    public void setCustomFeatures(){
// Optional: set other custom features of the TFOD Processor (4 are shown here).
        myTfodProcessorBuilder.setMaxNumRecognitions(1);  // Max. number of recognitions the network will return
        myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
        //myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
        myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track

    }

}
