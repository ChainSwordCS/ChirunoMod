#include <3ds.h>
#include <string.h>
#include <stdio.h>

#ifndef _HIMEM
Handle mcuHandle = 0;

Result mcuInit()
{
    return srvGetServiceHandle(&mcuHandle, "mcu::HWC");
}

Result mcuExit()
{
    return svcCloseHandle(mcuHandle);
}

Result mcuWriteRegister(u8 reg, void* data, u32 size)
{
    u32* ipc = getThreadCommandBuffer();
    ipc[0] = 0x20082;
    ipc[1] = reg;
    ipc[2] = size;
    ipc[3] = size << 4 | 0xA;
    ipc[4] = (u32)data;
    Result ret = svcSendSyncRequest(mcuHandle);
    if(ret < 0) return ret;
    return ipc[1];
}
#endif

int main()
{
    hidScanInput();
    u32 r = hidKeysHeld();
    u64 titleid;
    if(r & KEY_Y)
        titleid = 0x000401300CF00F02ULL; // HzMod
    else
        titleid = 0x000401300CF00902ULL; // ChirunoMod
    
#if _HIMEM
    
    srvPublishToSubscriber(0x204, 0);
    srvPublishToSubscriber(0x205, 0);
    
    while(aptMainLoop())
    {
        svcSleepThread(5e7);
    }
#endif
    
    nsInit();
    
#ifndef _HIMEM
    // shutdown HzMod and ChirunoMod if they happen to be running
    NS_TerminateProcessTID(0x000401300CF00F02ULL);
    NS_TerminateProcessTID(0x000401300CF00F09ULL);
#endif

#ifndef _HIMEM
    if(r & KEY_X) // Do this thing and don't boot
    {
        if(mcuInit() >= 0)
        {
            u8 blk[0x64];
            memset(blk, 0, sizeof(blk));
            blk[2] = 0xFF;
            mcuWriteRegister(0x2D, blk, sizeof(blk));
            mcuExit();
        }
    }
    else
#endif
    {
        u32 pid;
        Result ret;

        ret = NS_LaunchTitle(titleid, 0, &pid);

        if(ret < 0)
        {
            gfxInitDefault();
            consoleInit(GFX_BOTTOM, 0);
            printf("\nLaunchTitle (%i) failed: %08X\nPlease refer to 3DS error codes for details\n\nPress SELECT to exit", pid, ret);
            gfxFlushBuffers();
            while(aptMainLoop())
            {
                hidScanInput();
                if(hidKeysHeld() & KEY_SELECT) break;
            }
            gfxExit();
        }
    }
    
    nsExit();
    
    return 0;
}
