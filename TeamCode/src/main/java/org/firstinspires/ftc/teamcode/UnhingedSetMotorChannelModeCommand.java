package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.nio.ByteBuffer;

/**
 * Created by bob on 2016-03-07.
 */
public class UnhingedSetMotorChannelModeCommand extends LynxDekaInterfaceCommand<LynxAck>
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public final int cbPayload = 3;

    private byte motor;
    private byte mode;
    private byte floatAtZero;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public UnhingedSetMotorChannelModeCommand(LynxModuleIntf module)
    {
        super(module);
    }

    public UnhingedSetMotorChannelModeCommand(LynxModuleIntf module, int motorZ, DcMotor.RunMode mode, double floatOrBrake)
    {
        this(module);
        LynxConstants.validateMotorZ(motorZ);
        this.motor = (byte)motorZ;
        this.mode = LynxConstants.runModeToLynxMotorMode(mode);
        this.floatAtZero = (byte) floatOrBrake;
        Assert.assertTrue(floatOrBrake >= 0 && floatOrBrake <= 1);
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    public DcMotor.RunMode getMode()
    {
        switch (mode)
        {
            default:
            case 0: return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            case 1: return DcMotor.RunMode.RUN_USING_ENCODER;
            case 2: return DcMotor.RunMode.RUN_TO_POSITION;
        }
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior()
    {
        return floatAtZero==0 ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    @Override
    public byte[] toPayloadByteArray()
    {
        ByteBuffer buffer = ByteBuffer.allocate(cbPayload).order(LynxDatagram.LYNX_ENDIAN);
        buffer.put(this.motor);
        buffer.put(this.mode);
        buffer.put(this.floatAtZero);
        return buffer.array();
    }

    @Override
    public void fromPayloadByteArray(byte[] rgb)
    {
        ByteBuffer buffer = ByteBuffer.wrap(rgb).order(LynxDatagram.LYNX_ENDIAN);
        this.motor = buffer.get();
        this.mode = buffer.get();
        this.floatAtZero = buffer.get();
    }

    @Override
    public boolean isDangerous()
    {
        // The system needs to be able to send this command to set the zero-power behavior to FLOAT.
        // Because a failsafe command gets sent as soon as the OpMode is stopped, this command is not dangerous by itself.
        return false;
    }

}
