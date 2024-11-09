package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.nio.ByteBuffer;

/**
 * Created by bob on 2016-03-07.
 */
public class UnhingedSetMotorConstantPowerCommand extends LynxDekaInterfaceCommand<LynxAck>
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public final static int cbPayload = 3;
//    public final static int apiPowerLast  =  32767;
//    public final static int apiPowerFirst = -apiPowerLast;

    private byte motor;
    private short power;    // Power level (+CW, -CCW)

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public UnhingedSetMotorConstantPowerCommand(LynxModuleIntf module)
    {
        super(module);
    }

    public UnhingedSetMotorConstantPowerCommand(LynxModuleIntf module, int motorZ, int power)
    {
        this(module);
        LynxConstants.validateMotorZ(motorZ);
//        if (power < apiPowerFirst || power > apiPowerLast) throw new IllegalArgumentException(String.format("illegal power: %d", power));
        this.motor = (byte)motorZ;
        this.power = (short)power;
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    @Override
    public byte[] toPayloadByteArray()
    {
        ByteBuffer buffer = ByteBuffer.allocate(cbPayload).order(LynxDatagram.LYNX_ENDIAN);
        buffer.put(this.motor);
        buffer.putShort(this.power);
        return buffer.array();
    }

    @Override
    public void fromPayloadByteArray(byte[] rgb)
    {
        ByteBuffer buffer = ByteBuffer.wrap(rgb).order(LynxDatagram.LYNX_ENDIAN);
        this.motor = buffer.get();
        this.power = buffer.getShort();
    }

    @Override
    public boolean isDangerous()
    {
        return power != 0;
    }
}
