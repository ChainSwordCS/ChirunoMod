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

void bootChokiMod()
{
	
#if _HIMEM
    //printf("\n_HIMEM = true");
	
    srvPublishToSubscriber(0x204, 0);
    srvPublishToSubscriber(0x205, 0);
    
    while(aptMainLoop())
    {
        svcSleepThread(5e7);
    }
#endif
	
	//nsInit();
	
#ifndef _HIMEM
	//printf("\n_HIMEM = false");
    NS_TerminateProcessTID(0x000401300CF00F02ULL, 0); // "ULL"?
	//printf("\nTerminateProcess success");
    
    hidScanInput();
    if(hidKeysHeld() & (KEY_B))
    {
		//printf("\nB Button pressed");
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
		//printf("\nAttempting LaunchTitle");
        Result ret = NS_LaunchTitle(0x000401300CF00F02ULL, 0, &pid);
		if(ret > -1)
		{
			//printf("\nLaunchTitle success.");
		}
		else
        {
            //printf("\nLaunchTitle failed: %ld08\nPlease refer to 3DS error codes for details\n\nPress SELECT to exit", ret);
            while(aptMainLoop())
            {
                hidScanInput();
                if(hidKeysHeld() & KEY_SELECT)
					return;
            }
        }
    }
	return;
}

int main()
{
	nsInit();
	hidScanInput();
    if(hidKeysHeld() & KEY_SELECT)
	{
		//gfxInitDefault();
		//consoleInit(GFX_BOTTOM, 0);
		//printf("Hello world!\n\nWelcome to CHzLoad Debug Mode!\n\nTo proceed, press the button that corresponds to the desired option.\n\n    A - Boot CHokiMod\nSTART - Cancel and return to Home Menu");
		while(aptMainLoop())
		{
			hidScanInput();
			if(hidKeysHeld() & KEY_START) // Return to 3DS Home Menu (?)
			{
				//gfxExit();
				nsExit();
				return 0;
			}
			else if(hidKeysHeld() & KEY_A)
			{
				bootChokiMod();
				//gfxExit();
				nsExit();
				return 0;
			}
		}
	}
	else
	{
		//gfxInitDefault();
		//consoleInit(GFX_BOTTOM, 0);
		
		bootChokiMod();
		
		//gfxExit();
		nsExit();
		return 0;
	}
	// end of main function
}