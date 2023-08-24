#include <3ds.h>
#include <3ds/gpu/gx.h>

/*
    ChirunoMod - A utility background process for the Nintendo 3DS,
    purpose-built for screen-streaming over WiFi.

    Original HorizonM (HzMod) code is Copyright (C) 2017 Sono

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// this makes me so unfathomably upset. you can fix it and open a pull request if you wish
// i'd rather never think of this ever again. i hate makefile logic and syntax (and also my life)

#define DEBUG_BASIC 1
#define DEBUG_VERBOSE 0
// Unfinished
#define DEBUG_O3DSNEWINTERLACE 0
// Debug flag for testing; use experimental UDP instead of TCP.
// Don't enable this for now. It doesn't work.
#define DEBUG_USEUDP 0

extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <setjmp.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <malloc.h>
#include <errno.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/iosupport.h>

#include <poll.h>
#include <arpa/inet.h>

#include "miscdef.h"
//#include "service/screen.h"
#include "service/mcu.h"
#include "service/gx.h"
#include "misc/pattern.h"

#include "tga/targa.h"
#include <turbojpeg.h>
}

#include <exception>

#include "utils.hpp"



#define yield() svcSleepThread(1e8)

#define hangmacro()\
{\
    memset(&pat.r[0], 0x7F, 16);\
    memset(&pat.g[0], 0x7F, 16);\
    memset(&pat.b[0], 0x00, 16);\
    memset(&pat.r[16], 0, 16);\
    memset(&pat.g[16], 0, 16);\
    memset(&pat.b[16], 0, 16);\
    pat.ani = 0x1006;\
    PatApply();\
    while(1)\
    {\
        hidScanInput();\
        if(hidKeysHeld() == (KEY_SELECT | KEY_START))\
        {\
            goto killswitch;\
        }\
        yield();\
    }\
}

// Functions from original codebase.
int checkwifi();
int pollsock(int,int,int);
void CPPCrashHandler();


// Image Processing Functions added by me
void lazyConvert16to32andInterlace(u32,u32); // Finished, works. (Destructive implementation)
void convert16to24andInterlace(u32,u32); // Needs a rewrite (currently unused)
void fastConvert16to32andInterlace2_rgb565(u32,u32); // Finished, works. (Destructive implementation)
void convert16to24_rgb5a1(u32,u32); // Finished, works.
void convert16to24_rgb565(u32,u32); // Finished, works.
void convert16to24_rgba4(u32,u32); // Finished, works.
void dummyinterlace24(u32,u32); // Very unfinished, broken, do not use.

// Other big functions added by me
inline int setCpuResourceLimit(u32); // Unfinished, doesn't work.
void waitforDMAtoFinish(void*); // Don't use this. This is only used by a separate thread, started from within netfunc.
void sendDebugFrametimeStats(double,double,double*,double); // It works, and I'm still adding to it.

// Helper functions, added by me.
inline void cvt1632i_row1_rgb565(u32,u32*); // Unfinished
inline void cvt1632i_row2_rgb565(u32,u32*); // Unfinished
inline void cvt1624_help1(u32,u8**,u8**,u32);
inline void cvt1624_help2_forrgba4(u8*,u8*);
inline void cvt1624_help2_forrgb5a1(u8*,u8*);
inline void cvt1624_help2_forrgb565(u8*,u8*);

// netfunc
void netfuncOld3DS(void*); // Version of netfunc specifically for Old-3DS (code flow optimization)
void netfuncNew3DS(void*);

// Helper functions for netfunc 1 and 2 (code organization reasons)
inline void populatedmaconf(u8*,u32); // Rewritten from netfunc
inline int netfuncWaitForSettings(); // Rewritten from netfunc
inline void tryStopDma(); // Rewritten from netfunc (very small; shorthand because it's repeated)
inline void makeTargaImage(double*,double*,int,u32*,u32*,int*); // Rewritten from netfunc
inline void makeJpegImage(double*,double*,int,u32*,u32*,int*); // Rewritten from netfunc
inline void netfuncTestFramebuffer(u32*, int*); // Rewritten from netfunc

int main(); // So you can call main from main (:

static int haznet = 0;
int checkwifi()
{
    haznet = 0;
    u32 wifi = 0;
    hidScanInput();
    if(hidKeysHeld() == (KEY_SELECT | KEY_START)) return 0;
    if(ACU_GetWifiStatus(&wifi) >= 0 && wifi) haznet = 1;
    return haznet;
}


int pollsock(int sock, int wat, int timeout = 0)
{
    struct pollfd pd;
    pd.fd = sock;
    pd.events = wat;
    
    if(poll(&pd, 1, timeout) == 1)
        return pd.revents & wat;
    return 0;
}

const int bufsoc_pak_data_offset = 8; // After 8 bytes, the real data begins.

// Socket Buffer class
class bufsoc
{
public:
    
    typedef struct
    {
        //u32 packettype : 8;
        //u32 size : 24;
        u8 data[0];
    } packet;
    
    int socketid;
    u8* bufferptr;
    int bufsize;
    // recvsize is useless; is never read from.
    int recvsize;
    
    bufsoc(int passed_sock, int passed_bufsize)
    {
        bufsize = passed_bufsize;
        bufferptr = new u8[passed_bufsize];
        
        recvsize = 0;
        socketid = passed_sock;
    }
    
    // Destructor
    ~bufsoc()
    {
    	// If this socket buffer is already null,
    	// don't attempt to delete it again.
        if(!this) return;
        close(socketid);
        delete[] bufferptr;
    }
    
    // TODO: consider making these functions 'inline'

    u8 getPakType()
    {
    	return bufferptr[0];
    }

    u8 getPakSubtype()
    {
    	return bufferptr[1];
    }

    u8 getPakSubtypeB()
    {
    	return bufferptr[2];
    }

    // Retrieve the packet size, derived from the byte array
    u32 getPakSize()
    {
		return *( (u32*)(bufferptr+4) );
    }

    // Write to packet size, in the byte array
    void setPakSize(u32 input)
    {
    	*( (u32*)(bufferptr+4) ) = input;
    	return;
    }

    void setPakType(u8 input)
    {
    	bufferptr[0] = input;
    	return;
    }

    void setPakSubtype(u8 input)
    {
    	bufferptr[1] = input;
    	return;
    }

    void setPakSubtypeB(u8 input)
    {
    	bufferptr[2] = input;
    	return;
    }

    int avail()
    {
        return pollsock(socketid, POLLIN) == POLLIN;
    }
    
    int readbuf(int flags = 0)
    {
    	puts("attempting recv function call...");
        int ret = recv(socketid, bufferptr, 8, flags);
        printf("incoming packet type = %i\nsubtype1 = %i\nsubtype2 = %i\nrecv function return value = %i\n",bufferptr[0],bufferptr[1],bufferptr[2],ret);

        if(ret < 0) return -errno;
        if(ret < 8) return -1;

        u32 reads_remaining = getPakSize();
        printf("incoming packet size = %i\nrecv return value = %i\n",reads_remaining,ret);
        
        // Copy data to the buffer

        u32 offs = bufsoc_pak_data_offset; // Starting offset
        while(reads_remaining)
        {
            ret = recv(socketid, &(bufferptr[offs]), reads_remaining, flags);
            if(ret <= 0) return -errno;
            reads_remaining -= ret;
            offs += ret;
        }
        
        return offs;
    }
    
    int wribuf(int flags = 0)
    {
        u32 mustwri = getPakSize() + 8;
        int offs = 0;
        int ret = 0;
        
        while(mustwri)
        {
            if(mustwri >> 12)
                ret = send(socketid, &(bufferptr[offs]), 0x1000, flags);
            else
                ret = send(socketid, &(bufferptr[offs]), mustwri, flags);
            if(ret < 0) return -errno;
            mustwri -= ret;
            offs += ret;
        }
        
        return offs;
    }
    
    // Flags don't matter with UDP.
    // Pass the address of the "server" (client; PC; we are the server)
    // (Is this variable "sai" in most cases? I'll have to check)
    int wribuf_udp(struct sockaddr_in myservaddr)
    {
    	u32 mustwri = getPakSize() + 8;
    	int offs = 0;
    	int ret = 0;

    	while(mustwri)
    	{
    		u32 wri_now;
    		if(mustwri > 0x1000)
    		{
    			wri_now = 0x1000;
    		}
    		else
    		{
    			wri_now = mustwri;
    		}

    		// get host by name here (maybe) (unfinished)

    		ret = sendto(socketid, &(bufferptr[offs]), wri_now, 0, (struct sockaddr *)&myservaddr, sizeof(myservaddr));

    		if(ret < 0)
    			return -errno;

    		mustwri -= ret;
    		offs += ret;
    	}

    	return offs;
    }

    packet* pack()
    {
        return (packet*)bufferptr;
    }
    
    int errformat(char* c, ...)
    {
        //packet* p = pack();
        
        int len = 0;
        
        va_list args;
        va_start(args, c);

        // Is this line of code broken? I give up
        //len = vsprintf((char*)p->data + 1, c, args);
        len = vsprintf((char*)(bufferptr+bufsoc_pak_data_offset), c, args);

        va_end(args);
        
        if(len < 0)
        {
            puts("out of memory"); //???
            return -1;
        }
        
        //printf("Packet error %i: %s\n", p->packettype, p->data + 1);
        setPakType(0xFF);
        setPakSubtype(0x00);
        setPakSize(len); // Is "len + 2" necessary?
        
        return wribuf();
    }
};

static jmp_buf __exc;
static int  __excno;

void CPPCrashHandler()
{
    puts("\e[0m\n\n- The application has crashed\n\n");
    
    try
    {
        throw;
    }
    catch(std::exception &e)
    {
        printf("std::exception: %s\n", e.what());
    }
    catch(Result res)
    {
        printf("Result: %08X\n", res);
        //NNERR(res);
    }
    catch(int e)
    {
        printf("(int) %i\n", e);
    }
    catch(...)
    {
        puts("<unknown exception>");
    }
    
    puts("\n");
    
    PatStay(0xFFFFFF);
    PatPulse(0xFF);
    
    svcSleepThread(1e9);
    
    hangmacro();
    
    killswitch:
    longjmp(__exc, 1);
}


extern "C" u32 __get_bytes_per_pixel(GSPGPU_FramebufferFormats format);

const int port = 6464;

static u32 kDown = 0;
static u32 kHeld = 0;
static u32 kUp = 0;

// GPU 'Capture Info' object.
// Data about framebuffers from the GSP.
static GSPGPU_CaptureInfo capin;

// Whether or not this is running on an 'Old' 3DS?
static int isold = 1;

static Result ret = 0;

// Related to screen capture. Dimensions and color format.
static u32 offs[2] = {0, 0};
static u32 limit[2] = {1, 1};
static u32 stride[2] = {80, 80};
static u32 format[2] = {0xF00FCACE, 0xF00FCACE};

// Config Block
static u8 cfgblk[0x100];

// Set to 1 to force Interlaced
// Set to -1 to force Non-Interlaced
// Set to 0 to not force anything
//
// This is only listened to by a few select bits of code
// Intended for when either Interlaced or Progressive mode
// is not yet implemented for a given color format.
static int forceInterlaced = 0;

static int sock = 0;

static struct sockaddr_in sai;
static socklen_t sizeof_sai = sizeof(sai);

static bufsoc* soc = nullptr;

static bufsoc::packet* k = nullptr;

static Thread netthread = 0;
static vu32 threadrunning = 0;

static u32* screenbuf = nullptr;

// Obsolete now. Will be removed eventually.
//static u8 pxarraytwo[2*120*400];
static u8* pxarraytwo = nullptr;

static tga_image img;
static tjhandle jencode = nullptr;

static TickCounter tick_ctr_1;
static TickCounter tick_ctr_2_dma;

// If this is 0, we are converting odd-numbered rows.
// If this is 2, we are converting even-numbered rows.
//
// This variable is separate from the row index numbers.
// Rows of pixels will generally be indexed starting at 1 and ending at 240 (or 120)
static int interlace_px_offset = 0;

// Super lazy, proof-of-concept function for converting 16-bit color images to 24-bit.
// (Note: This function focuses on speed and not having to reallocate anything.)
// Converts to RGBA8 (hopefully) (TJPF_RGBX)
//
// As it is now, Alpha is not read and will always be totally nullified after this function.
// Interpreting the resulting data as RGBA instead of RGBX could cause the
// Alpha channel to be interpreted as 0.
//
// P.S. Sorry my code is dumb and bad and hard to read lol
//
// Note to self... What I actually kinda want to do is to modify the libturbojpeg code
// to accept 16bpp input to the compressor, but that'd be a whole other challenge...
//
void lazyConvert16to32andInterlace(u32 flag, u32 passedsiz)
{
	// offs is used to track our progress through the loop.
	u32 offs = 0;

	// One-time reinterpret as an array of u8 objects
	// u8scrbuf points to the exact same data (if my logic is sound)
	u8* u8scrbuf = (u8*)screenbuf;
	//u16* u16scrbuf = (u16*)screenbuf;

	if(flag == 4) // RGBA4 -> RGBA8
	{
		while((offs + 3) < passedsiz) // This conditional should be good enough to catch errors...
		{
			u8 b = ( u8scrbuf[offs + interlace_px_offset] & 0b11110000);
			u8 g = ( u8scrbuf[offs+1+interlace_px_offset] & 0b00001111) << 4;
			u8 r = ( u8scrbuf[offs+1+interlace_px_offset] & 0b11110000);
			screenbuf[(offs/4)] = ((u32)r << 16) + ((u32)g << 8) + ((u32)b << 0);

			// At compile-time, hopefully this will just be one register(?)
			// i.e. this is hard to read, but I think working with u32 objects instead of u8s will save us CPU time(...?)

			// derive red pixel
			//u32 rgba8pix = (u32)(u8scrbuf[(offs+interlace_px_offset)] & 0b11110000) << 24;
			// derive green pixel
			//rgba8pix = rgba8pix + ( (u32)(u8scrbuf[(offs+interlace_px_offset)] & 0b00001111) << 20 );
			// derive blue pixel
			//rgba8pix = rgba8pix + ( (u32)(u8scrbuf[(offs+1+interlace_px_offset)] & 0b11110000) << 8 );
			//screenbuf[(offs/4)] = rgba8pix;

			offs = offs + 4;
		}
	}
	else if(flag == 3) // RGB5A1 -> RGBA8
	{
		while((offs + 3) < passedsiz)
		{
			u8 b = ( u8scrbuf[offs + interlace_px_offset] & 0b00111110) << 2;
			u8 g = ( u8scrbuf[offs + interlace_px_offset] & 0b11000000) >> 3;
			g = g +((u8scrbuf[offs+1+interlace_px_offset] & 0b00000111) << 5);
			u8 r = ( u8scrbuf[offs+1+interlace_px_offset] & 0b11111000);
			screenbuf[(offs/4)] = ((u32)r << 16) + ((u32)g << 8) + ((u32)b << 0);

			offs = offs + 4;
		}
	}
	else if(flag == 2) // RGB565 -> RGBA8
	{
		while((offs + 3) < passedsiz)
		{
			u8 b = ( u8scrbuf[offs + interlace_px_offset] & 0b00011111) << 3;
			u8 g = ( u8scrbuf[offs + interlace_px_offset] & 0b11100000) >> 3;
			g = g +((u8scrbuf[offs+1+interlace_px_offset] & 0b00000111) << 5);
			u8 r = ( u8scrbuf[offs+1+interlace_px_offset] & 0b11111000);
			screenbuf[(offs/4)] = ((u32)r << 16) + ((u32)g << 8) + ((u32)b << 0);

			offs = offs + 4;
		}
	}
	else
	{
		// Do nothing; we expect to receive a valid flag.
	}

	// Next frame, do the other set of rows instead.
	if(interlace_px_offset == 0)
		interlace_px_offset = 2;
	else
		interlace_px_offset = 0;

	return;
}

// Needs a rewrite.
void convert16to24andInterlace(u32 flag, u32 passedsiz)
{
	u32 offs_univ = 0;
	const u32 ofumax = 120*400;

	//const u32 buf2siz = 2*120*400;

	u8* u8scrbuf = (u8*)screenbuf;

	if(interlace_px_offset == 0)
	{
		if(flag == 4) // RGBA4 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs1 = offs_univ*4;

				u8 r = ( u8scrbuf[deriveoffs1+0] & 0b11110000);
				u8 g = ( u8scrbuf[deriveoffs1+1] & 0b00001111) << 4;
				u8 b = ( u8scrbuf[deriveoffs1+1] & 0b11110000);

				u32 deriveoffs2 = offs_univ*2;

				pxarraytwo[deriveoffs2+0] = u8scrbuf[deriveoffs1+2];
				pxarraytwo[deriveoffs2+1] = u8scrbuf[deriveoffs1+3];

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else if(flag == 3) // RGB5A1 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs1 = offs_univ*4;

				u8 r = ( u8scrbuf[deriveoffs1+0] & 0b00111110) << 2;
				u8 g = ( u8scrbuf[deriveoffs1+0] & 0b11000000) >> 3;
				g = g +((u8scrbuf[deriveoffs1+1] & 0b00000111) << 5);
				u8 b = ( u8scrbuf[deriveoffs1+1] & 0b11111000);

				u32 deriveoffs2 = offs_univ*2;

				pxarraytwo[deriveoffs2+0] = u8scrbuf[deriveoffs1+2];
				pxarraytwo[deriveoffs2+1] = u8scrbuf[deriveoffs1+3];

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else if(flag == 2) // RGB565 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs1 = offs_univ*4;

				u8 r = ( u8scrbuf[deriveoffs1+0] & 0b00011111) << 3;
				u8 g = ( u8scrbuf[deriveoffs1+0] & 0b11100000) >> 3;
				g = g +((u8scrbuf[deriveoffs1+1] & 0b00000111) << 5);
				u8 b = ( u8scrbuf[deriveoffs1+1] & 0b11111000);

				u32 deriveoffs2 = offs_univ*2;

				pxarraytwo[deriveoffs2+0] = u8scrbuf[deriveoffs1+2];
				pxarraytwo[deriveoffs2+1] = u8scrbuf[deriveoffs1+3];

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else
		{
			// Do nothing; we expect to receive a valid flag.
		}
		interlace_px_offset = 2;
	}
	else
	{
		// Alternate rows. Complex style...

		if(flag == 4) // RGBA4 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs2 = offs_univ*2;

				u8 r = ( pxarraytwo[deriveoffs2+0] & 0b11110000);
				u8 g = ( pxarraytwo[deriveoffs2+0] & 0b00001111) << 4;
				u8 b = ( pxarraytwo[deriveoffs2+1] & 0b11110000);

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else if(flag == 3) // RGB5A1 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs2 = offs_univ*2;

				u8 r = ( pxarraytwo[deriveoffs2+0] & 0b00111110) << 2;
				u8 g = ( pxarraytwo[deriveoffs2+0] & 0b11000000) >> 3;
				g = g +((pxarraytwo[deriveoffs2+1] & 0b00000111) << 5);
				u8 b = ( pxarraytwo[deriveoffs2+1] & 0b11111000);

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else if(flag == 2) // RGB565 -> RGB8
		{
			while(offs_univ < ofumax) // (offs + 2) < passedsiz && (offstwo+1) < buf2siz )
			{
				u32 deriveoffs2 = offs_univ*2;

				u8 r = ( pxarraytwo[deriveoffs2+0] & 0b00011111) << 3;
				u8 g = ( pxarraytwo[deriveoffs2+0] & 0b11100000) >> 3;
				g = g +((pxarraytwo[deriveoffs2+1] & 0b00000111) << 5);
				u8 b = ( pxarraytwo[deriveoffs2+1] & 0b11111000);

				u32 deriveoffs3 = offs_univ*3;

				u8scrbuf[deriveoffs3] = r;
				u8scrbuf[deriveoffs3+1] = g;
				u8scrbuf[deriveoffs3+2] = b;

				offs_univ++;
			}
		}
		else
		{
			// Do nothing; we expect to receive a valid flag.
		}
		interlace_px_offset = 0;
	}

	return;
}

// Unreadable code alert
inline void cvt1632i_row1_rgb565(u32 pxnum, u32* fbuf)
{
	u32 temppx = fbuf[pxnum];
	// Blue
	fbuf[pxnum] =(temppx & 0x0000F800) << 8;
	// Green
	fbuf[pxnum]+=(temppx & 0x000007E0) << 5;
	// Red
	fbuf[pxnum]+=(temppx & 0x0000001F) << 3;
}

inline void cvt1632i_row2_rgb565(u32 pxnum, u32* fbuf)
{
	u32 temppx = fbuf[pxnum];
	// Blue
	fbuf[pxnum] =(temppx & 0xF8000000) >> 8;
	// Green
	fbuf[pxnum]+=(temppx & 0x07E00000) >> 11; // 8 + 3
	// Red
	fbuf[pxnum]+=(temppx & 0x001F0000) >> 13; // 8 + 5
}

// Unfinished, colors are broken.
// (The bottleneck may be elsewhere right now. -C 2022-09-03)
//
// Note: Destructive implementation. Every other row is trashed and not saved.
// Therefore it is necessary to DMA the framebuffer again every time we do this.
// It's a tradeoff, it is what it is, but it's fast(ish) baby
void fastConvert16to32andInterlace2_rgb565(u32 stride, u32 myscrw)
{
	u32 i = 0;
	u32 max = (myscrw/2)*stride;
	u32* fbuf = screenbuf; // This is just always this value. lol

	if(interlace_px_offset == 0)
	{
		while(i < max)
		{
			cvt1632i_row1_rgb565(i,fbuf);
			i++;
		}
		interlace_px_offset = 2;
	}
	else
	{
		while(i < max)
		{
			cvt1632i_row2_rgb565(i,fbuf);
			i++;
		}
		interlace_px_offset = 0;
	}
}

// Helper function 1
inline void cvt1624_help1(u32 mywidth, u8** endof24bimg, u8** endof16bimg, u32 myscrw)
{
	*endof24bimg = (u8*)screenbuf + (myscrw*mywidth*3) - 3; // -1
	*endof16bimg = (u8*)screenbuf + (myscrw*mywidth*2) - 2; // -1
	//*sparebuffersiz = (mywidth*240*4) - (mywidth*240*3);
}

inline void cvt1624_help2_forrgba4(u8* myaddr1, u8* myaddr2)
{
	u8 r = myaddr1[0] & 0b11110000;
	u8 g =(myaddr1[1] & 0b00001111) << 4;
	u8 b = myaddr1[1] & 0b11110000;

	myaddr2[0] = r;
	myaddr2[1] = g;
	myaddr2[2] = b;
}

inline void cvt1624_help2_forrgb5a1(u8* myaddr1, u8* myaddr2)
{
	u8 r =(myaddr1[0] & 0b00111100) << 2;
	u8 g =(myaddr1[0] & 0b11000000) >> 3;
	   g+=(myaddr1[1] & 0b00000111) << 5;
	u8 b = myaddr1[1] & 0b11111000;

	myaddr2[0] = r;
	myaddr2[1] = g;
	myaddr2[2] = b;
}

inline void cvt1624_help2_forrgb565(u8* myaddr1, u8* myaddr2)
{
	u8 r =(myaddr1[0] & 0b00011111) << 3;
	u8 g =(myaddr1[0] & 0b11100000) >> 3;
	   g+=(myaddr1[1] & 0b00000111) << 5;
	u8 b = myaddr1[1] & 0b11111000;

	myaddr2[0] = r;
	myaddr2[1] = g;
	myaddr2[2] = b;
}

// This is Progressive! (Not Interlaced)
// Also, this works on Old-3DS.
void convert16to24_rgb5a1(u32 scrbfwidth, u32 myscrw)
{
	u8* buf16; // Copy FROM here, starting at the end of the 16bpp buffer
	u8* buf24; // Copy TO here, starting at the end of the 24bpp buffer
	cvt1624_help1(scrbfwidth, &buf24, &buf16, myscrw); // calc variables

	while(buf16 + 1 < buf24)
	{
		cvt1624_help2_forrgb5a1(buf16,buf24);
		buf16 -= 2;
		buf24 -= 3;
	}
}

// Progressive (Not Interlaced)
// Works on Old-3DS
void convert16to24_rgb565(u32 scrbfwidth, u32 myscrw)
{
	u8* buf16;
	u8* buf24;
	cvt1624_help1(scrbfwidth, &buf24, &buf16, myscrw);

	while(buf16 + 1 < buf24)
	{
		cvt1624_help2_forrgb565(buf16,buf24);
		buf16 -= 2;
		buf24 -= 3;
	}
}

// Progressive (Not Interlaced)
// Works on Old-3DS
void convert16to24_rgba4(u32 scrbfwidth, u32 myscrw)
{
	u8* buf16;
	u8* buf24;
	cvt1624_help1(scrbfwidth, &buf24, &buf16, myscrw);

	while(buf16 + 1 < buf24)
	{
		cvt1624_help2_forrgba4(buf16,buf24);
		buf16 -= 2;
		buf24 -= 3;
	}
}

void dummyinterlace24(u32 passedsiz, u32 scrbfwidth) // UNFINISHED
{
	u32 offs2 = 0;

	// Used as a spare buffer.
	// This is critical on Old-3DS,
	// (This is only used when we have a 32bpp image,
	// so it's free memory otherwise! :)
	//
	// Buffer starts at refaddr_endof24bimg + 1
	u32 sparebuffersiz;

	u8* refaddr_endof24bimg;
	u8* refaddr_endof16bimg;
	//cvt1624_help1(scrbfwidth, &refaddr_endof24bimg, &refaddr_endof16bimg, &sparebuffersiz);

	// Address to read from, the first of two bytes of
	// the very last pixel of the 16bpp image.
	u8* addr1 = refaddr_endof16bimg - 1;
	// Address to write to, the first of three bytes of
	// the very last pixel of the 24bpp image.
	u8* addr2 = refaddr_endof24bimg - 2;

	//u32* addr3 = 0;

	u8* addr4 = 0;

	u32 pixelsdrawn = 0;
	u32 maxpix = scrbfwidth * 240;

	// this While-loop is only Part 1.
	// When these two addresses are too close together,
	// we move on to Part 2.
	while(addr1 + 1 < addr2 && addr1 >= (u8*)screenbuf)
	{
		cvt1624_help2_forrgb5a1(addr1,addr2);

		// Increment and decrement
		pixelsdrawn++;
		addr1 -= 2;
		addr2 -= 3;
	}

	// Big Part 2
	while(false)//(pixelsdrawn <= maxpix)
	{
		offs2 = 0;
		addr4 = refaddr_endof24bimg + 1;

		// Copy from 16bpp framebuffer to spare buffer
		while(addr1 >= (u8*)screenbuf && pixelsdrawn < maxpix && addr4 <= (refaddr_endof24bimg+sparebuffersiz))
		{
			//cvt1624_help3(addr4,addr1);

			addr1 -= 2;
			addr4 += 2;
		}

		if(addr1 < (u8*)screenbuf)
			addr1 = (u8*)screenbuf;

		offs2 = 0;
		addr4 = refaddr_endof24bimg + 1;

		// Copy from spare buffer to 24bpp framebuffer
		while(addr1 <= addr2 && offs2 + 1 < sparebuffersiz && pixelsdrawn < maxpix)
		{
			cvt1624_help2_forrgb5a1((u8*)addr4,(u8*)addr2);
			// Increment
			addr2 -= 3;
			offs2 += 2;
			addr4 += 2;
			pixelsdrawn++;
		}

		// Loop back on Part 2 (:
	}

	// return;
}

// Based on (and slightly modified from) devkitpro/libctru source
//
// svcSetResourceLimitValues(Handle res_limit, LimitableResource* resource_type_list, s64* resource_list, u32 count)
//
// Note : These aren't in 2017-libctru. I'm outta luck until compatibility with current libctru is fixed.
//
inline int setCpuResourceLimit(u32 cpu_time_limit)
{
	int ret = 0;

	Handle reslimithandle;
	ret = svcGetResourceLimit(&reslimithandle,0xFFFF8001);

	if(ret<0)
		return ret;

	//ResourceLimitType name = RESLIMIT_CPUTIME;
	s64 value = cpu_time_limit;
	//ret = svcSetResourceLimitValues(reslimithandle, &name, &value, 1);

	if(ret < 0)
		return ret;

	//ret = svcSetProcessResourceLimits(0xFFFF8001, reslimithandle);

	return ret;
}

double timems_dmaasync = 0;
u32 dmastatusthreadrunning = 0;
u32 dmafallbehind = 0;
Handle dmahand = 0;

void waitforDMAtoFinish(void* __dummy_arg__)
{
	// Don't have more than one thread running this
	// function at a time. Don't want to accidentally
	// overload and slow the 3DS.
	dmastatusthreadrunning = 1;

	int r1 = 0;

	//while(r1 != 4)// DMASTATE_DONE)
	//{
		//svcSleepThread(1e7); // 10 ms
		//r2 = svcGetDmaState(&r1,dmahand);
	//}


	r1 = svcWaitSynchronization(dmahand,500000); // keep trying and waiting for half a second

	if(r1 >= 0)
	{
		osTickCounterUpdate(&tick_ctr_2_dma);
		timems_dmaasync = osTickCounterRead(&tick_ctr_2_dma);
	}

	dmastatusthreadrunning = 0;
	return;
}

void sendDebugFrametimeStats(double ms_compress, double ms_writesocbuf, double* ms_dma, double ms_convert)
{
	const u32 charbuflimit = 100 + sizeof(char); // ? I don't even know what this means but whatever.
	char str1[charbuflimit];
	char str2[charbuflimit];
	char str3[charbuflimit];
	char str4[charbuflimit];

	sprintf(str4,"Image format conversion / interlacing took %g ms\n",ms_convert);
	sprintf(str1,"Image compression took %g ms\n",ms_compress);
	sprintf(str2,"Copying to Socket Buffer (in WRAM) took %g ms\n",ms_writesocbuf);

	if(*ms_dma == 0)
	{
		sprintf(str3,"DMA not yet finished\n");
		dmafallbehind++;
	}
	else
	{
		double ms_dma_localtemp = *ms_dma;
		*ms_dma = 0;
		sprintf(str3,"DMA copy from framebuffer to ChirunoMod WRAM took %g ms (measurement is %i frames behind)\n",ms_dma_localtemp,dmafallbehind);
		dmafallbehind = 0;
	}

	soc->setPakType(0xFF);
	soc->setPakSubtype(03);

	char finalstr[500+sizeof(char)];

	u32 strsiz = sprintf(finalstr,"%s%s%s%s",str4,str1,str2,str3);

	strsiz--;

	for(u32 i=0; i<strsiz; i++)
	{
		((char*)soc->bufferptr + bufsoc_pak_data_offset)[i] = finalstr[i];
	}

	soc->setPakSize(strsiz);
	soc->wribuf();
	return;
}

inline void populatedmaconf(u8* dmac, u32 flag)
{
	// Note: in modern libctru, DmaConfig is its own object type.
	// https://www.3dbrew.org/wiki/Corelink_DMA_Engines
	// https://github.com/devkitPro/libctru/blob/master/libctru/include/3ds/svc.h

	if(flag == 1) // Use custom, non-standard DMA config. (This specific config is for interlaced, but it's not very necessary...)
	{
		//dmac[1] = 0; // Endian swap size. 0 = None, 2 = 16-bit, 4 = 32-bit, 8 = 64-bit
		dmac[2] = 0b11000000; // Flags. DMACFG_USE_SRC_CONFIG and DMACFG_USE_DST_CONFIG
		//dmaconf[3] = 0; // Padding.

		// Destination Config block
		dmac[4] = 0xFF; // peripheral ID. FF for ram (it's forced to FF anyway)
		dmac[5] = 8|4|2|1; // Allowed Alignments. Defaults to "1|2|4|8" (15). Also acceptable = 4, 8, "4|8" (12)
		*(u16*)(dmac+6) = 3;// Not exactly known...
		*(u16*)(dmac+8) = 3; // Not exactly known...
		*(u16*)(dmac+10) = 6; // Number of bytes transferred at once(?)
		*(u16*)(dmac+12) = 6; // Number of bytes transferred at once(?) (or Stride)

		// Source Config block
		dmac[14] = 0xFF; // Peripheral ID
		dmac[15] = 8|4|2|1; // Allowed Alignments (!)
		*(u16*)(dmac+16) = 0x0003;//x80; // burstSize? (Number of bytes transferred in a burst loop. Can be 0, in which case the max allowed alignment is used as a unit.)
		*(u16*)(dmac+18) = 0x0003;//x80; // burstStride? (Burst loop stride, can be <= 0.
		*(u16*)(dmac+20) = 6; // transferSize? (Number of bytes transferred in a "transfer" loop, which is made of burst loops.)
		*(u16*)(dmac+22) = 6; // transferStride? ("Transfer" loop stride, can be <= 0.)
	}
	else
	{
		memset(dmac, 0, 0x18);
	}

	dmac[0] = -1; // -1 = Auto-assign to a free channel (Arm11: 3-7, Arm9:0-1)

	return;
}

// Returns -1 on an error, and expects the calling function to close the socket.
// Returns 1 on success.
inline int netfuncWaitForSettings()
{
	while(1)
	{
		if((kHeld & (KEY_SELECT | KEY_START)) == (KEY_SELECT | KEY_START))
			return -1;

#if DEBUG_BASIC==1
		puts("Reading incoming packet...");
#endif

		int r = soc->readbuf();
		if(r <= 0)
		{
#if DEBUG_BASIC==1
			printf("Failed to recvbuf: (%i) %s\n", errno, strerror(errno));
#endif
			return -1;
		}
		else
		{
			u8 i = soc->getPakSubtype();
			u8 j = soc->bufferptr[bufsoc_pak_data_offset];
			// Only used in one of these, but want to be declared up here.
			u32 k;
			u32 l;

			switch(soc->getPakType())
			{
				case 0x02: // Init (New CHmod / CHokiMod Packet Specification)
					cfgblk[0] = 1;
					// TODO: Maybe put sane defaults in here, or in the variable init code.
					return 1;

				case 0x03: // Disconnect (new packet spec)
					cfgblk[0] = 0;
#if DEBUG_BASIC==1
					puts("forced dc");
#endif
					return -1;

				case 0x04: // Settings input (new packet spec)

					switch(i)
					{
						case 0x01: // JPEG Quality (1-100%)
							// Error-Checking
							if(j > 100)
								cfgblk[1] = 100;
							else if(j < 1)
								cfgblk[1] = 1;
							else
								cfgblk[1] = j;
							return 1;

						case 0x02: // CPU Cap value / CPU Limit / App Resource Limit

							// Redundancy check
							if(j == cfgblk[2])
								return 1;

							// Maybe this is percentage of CPU time? (https://www.3dbrew.org/wiki/APT:SetApplicationCpuTimeLimit)
							// In which case, values can range from 5% to 89%
							// (The respective passed values are 5 and 89, respectively)
							// So I don't know if 0x7F (127) will work.
							//
							// Maybe I'm looking at two different things by accident.

							// Also, it may be required to set the 'reslimitdesc' in exheader a certain way (in cia.rsf)

							if(j > 0x7F)
								j = 0x7F;
							else if(j < 5)
								j = 5;

							// This code doesn't work, lol.
							// Functionality dummied out for now.
							//setCpuResourceLimit((u32)j);

							cfgblk[2] = j;

							return 1;

						case 0x03: // Which Screen
							if(j < 1 || j > 3)
								cfgblk[3] = 1; // Default to top screen only
							else
								cfgblk[3] = j;
							return 1;

						case 0x04: // Image Format (JPEG or TGA?)
							if(j > 1)
								cfgblk[4] = 0;
							else
								cfgblk[4] = j;
							return 1;

						case 0x05: // Request to use Interlacing (yes or no)
							if(j == 0)
								cfgblk[5] = 0;
							else
								cfgblk[5] = 1;
							return 1;

						case 0x06: // Force hotfix for Mario Kart 7 (on or off; breaks compatibility with all other games)
							if(j == 0)
								cfgblk[6] = 0;
							else
								cfgblk[6] = 1;
							return 1;

						default:
							// Invalid subtype for "Settings" packet-type
							return 1;
					}
					return 1; // Just in case?

				case 0xFF: // Debug info. Prints to log file, interpreting the Data as u8 char objects.
					// Note: packet subtype is ignored, lol.
#if DEBUG_BASIC==1
					k = soc->getPakSize();
					// Current offset
					l = 0;

					if(k > 255) // Error checking; arbitrary limit on text characters.
						k = 255;

					while(k > 0)
					{
						printf((char*)(soc->bufferptr + bufsoc_pak_data_offset));
						k--;
						l++;
					}
#endif
					return 1;

				default:
#if DEBUG_BASIC==1
					printf("Invalid packet ID: %i\n", soc->getPakType());
#endif
					return -1;
			}

			return 1;
		}
	}

	return 1;
}

inline void tryStopDma()
{
	if(dmahand)
	{
		svcStopDma(dmahand);
		svcCloseHandle(dmahand);
		dmahand = 0;
	}
}

inline void makeTargaImage(double* timems_fc, double* timems_pf, int scr, u32* scrw, u32* bits, int* imgsize)
{
#if DEBUG_VERBOSE==1
	*timems_fc = 0;
	osTickCounterUpdate(&tick_ctr_1);
#endif

	u32 bits2 = *bits;

    switch(format[scr] & 0b111)
	{
    	case 0:
    		bits2 = 32;
    		break;
    	case 1:
    		bits2 = 24;
    		break;
		case 2: // RGB565
			bits2 = 17;
			break;
		case 3: // RGB5A1
			bits2 = 16;
			break;
		case 4: // RGBA4
			bits2 = 18;
			break;
		default:
			break;
	}

	// Note: interlacing not yet implemented here.
	init_tga_image(&img, (u8*)screenbuf, *scrw, stride[scr], bits2);
	img.image_type = TGA_IMAGE_TYPE_BGR_RLE;
	img.origin_y = (scr * 400) + (stride[scr] * offs[scr]);
	tga_write_to_FILE((soc->bufferptr + bufsoc_pak_data_offset), &img, imgsize);

#if DEBUG_VERBOSE==1
	osTickCounterUpdate(&tick_ctr_1);
	*timems_pf = osTickCounterRead(&tick_ctr_1);
#endif

	u8 subtype_aka_flags = 0b00001000 + (scr * 0b00010000) + (format[scr] & 0b111);
	soc->setPakType(01); // Image
	soc->setPakSubtype(subtype_aka_flags);
	soc->setPakSize(*imgsize);

	return;
}

inline void makeJpegImage(double* timems_fc, double* timems_pf, int scr, u32* scrw, u32* bsiz, int* imgsize)
{
	u32 f = format[scr] & 0b111;
	u8 subtype_aka_flags = 0b00000000 + (scr * 0b00010000) + f;
	int tjpf = 0;
	u32 siz_2 = (capin.screencapture[scr].framebuf_widthbytesize * stride[scr]);

#if DEBUG_VERBOSE==1
	osTickCounterUpdate(&tick_ctr_1);
#endif

	switch(f)
	{
		case 0: // RGBA8
			forceInterlaced = -1; // Function not yet implemented
			tjpf = TJPF_RGBX;
			*bsiz = 4;
			break;

		case 1: // RGB8
			forceInterlaced = -1; // Function not yet implemented
			tjpf = TJPF_RGB;
			*bsiz = 3;
			break;

		case 2: // RGB565
			forceInterlaced = 0;
			if(cfgblk[5] == 1) // Interlaced
			{
				fastConvert16to32andInterlace2_rgb565(stride[scr], *scrw);
				//lazyConvert16to32andInterlace(2,siz_2);
				tjpf = TJPF_RGBX;
				*bsiz = 4;
				*scrw = *scrw / 2;
				//*scrw = *scrw / 2; // Account for Mario Kart 7. Usually 120, but in that one case it's 128.
				subtype_aka_flags += 0b00100000 + (interlace_px_offset?0:0b01000000);
			}
			else
			{
				convert16to24_rgb565(stride[scr], *scrw);
				tjpf = TJPF_RGB;
				*bsiz = 3;
			}
			break;

		case 3: // RGB5A1
			forceInterlaced = 0;
			if(cfgblk[5] == 1) // Interlaced
			{
				lazyConvert16to32andInterlace(3,siz_2);
				tjpf = TJPF_RGBX;
				*bsiz = 4;
				*scrw = *scrw / 2;
				//*scrw = *scrw / 2; // MK7
				subtype_aka_flags += 0b00100000 + (interlace_px_offset?0:0b01000000);
			}
			else
			{
				convert16to24_rgb5a1(stride[scr], *scrw);
				tjpf = TJPF_RGB;
				*bsiz = 3;
			}
			break;

		case 4: // RGBA4
			forceInterlaced = 0;
			if(cfgblk[5] == 1) // Interlaced
			{
				lazyConvert16to32andInterlace(4,siz_2);
				tjpf = TJPF_RGBX;
				*bsiz = 4;
				*scrw = *scrw / 2;
				//*scrw = *scrw / 2; // MK7
				subtype_aka_flags += 0b00100000 + (interlace_px_offset?0:0b01000000);
			}
			else
			{
				convert16to24_rgba4(stride[scr], *scrw);
				tjpf = TJPF_RGB;
				*bsiz = 3;
			}
			break;

		default:
			// Invalid format, should never happen, but put a failsafe here anyway.
			//
			// This failsafe is just taken from the 24-bit code. I don't know if that's the
			// safest or not, it's just a placeholder. -C
			tjpf = TJPF_RGB;
			//*bsiz = 3;
			//*scrw = 240;
			forceInterlaced = -1;
			break;
	}

#if DEBUG_VERBOSE==1
	osTickCounterUpdate(&tick_ctr_1);
	*timems_fc = osTickCounterRead(&tick_ctr_1);
#endif

	// TODO: Important!
	// For some unknown reason, Mario Kart 7 requires the "width" (height)
	// to be 128 when interlaced. And possibly 256 or something similar
	// when not interlaced. No I don't know why.
	// But I would love to get to the bottom of it.
	// If I can't, I'll add a debug feature to force-override the number.

	// Experimental option: to try and save time, don't even keep a framebuffer ourselves (but this doesn't save much time in practice)
	//u8* experimentaladdr1 = (u8*)capin.screencapture[scr].framebuf0_vaddr + (siz * offs[scr]);
	u8* experimentaladdr1 = (u8*)screenbuf;

	u8* destaddr = soc->bufferptr + bufsoc_pak_data_offset;

	if(!tjCompress2(jencode, experimentaladdr1, *scrw, (*bsiz) * (*scrw), stride[scr], tjpf, &destaddr, (u32*)imgsize, TJSAMP_420, cfgblk[1], TJFLAG_NOREALLOC | TJFLAG_FASTDCT))
	{
#if DEBUG_VERBOSE==1
		osTickCounterUpdate(&tick_ctr_1);
		*timems_pf = osTickCounterRead(&tick_ctr_1);
#endif
		soc->setPakSize(*imgsize);
	}
	else
	{
#if DEBUG_VERBOSE==1
		*timems_pf = 0;
#endif
	}

	soc->setPakType(01); //Image
	soc->setPakSubtype(subtype_aka_flags);
	return;
}

// This is implemented in a really dumb way i think
inline void tryMarioKartHotfix(u32* scrw)
{
	if(cfgblk[6] == 1)
	{
		*scrw = 256;
	}
	else
	{
		*scrw = 240;
	}
}

inline void netfuncTestFramebuffer(u32* procid, int* scr)
{
	//test for changed framebuffers
	if(capin.screencapture[0].format != format[0] || capin.screencapture[1].format != format[1])
	{
		PatStay(0xFFFF00); // Notif LED = Teal

		format[0] = capin.screencapture[0].format;
		format[1] = capin.screencapture[1].format;

		tryStopDma();

		*procid = 0;

		//test for VRAM
		if( (u32)capin.screencapture[0].framebuf0_vaddr >= 0x1F000000 && (u32)capin.screencapture[0].framebuf0_vaddr <  0x1F600000 )
		{
			// nothing to do?
			// If the framebuffer is in VRAM, we don't have to do anything special(...?)
			// (Such is the case for all retail applets, apparently.)
		}
		else //use APT fuckery, auto-assume this is an application
		{
			// Notif LED = Flashing red and green
			memset(&pat.r[0], 0xFF, 16);
			memset(&pat.r[16], 0, 16);
			memset(&pat.g[0], 0, 16);
			memset(&pat.g[16], 0xFF, 16);
			memset(&pat.b[0], 0, 32);
			pat.ani = 0x2004;
			PatApply();

			u64 progid = -1ULL;
			bool loaded = false;

			while(1)
			{
				// loaded = Registration Status(?) of the specified application.
				loaded = false;
				while(1)
				{
					if(APT_GetAppletInfo((NS_APPID)0x300, &progid, nullptr, &loaded, nullptr, nullptr) < 0)
						break;
					if(loaded)
						break;
					svcSleepThread(15e6);
				}
				if(!loaded)
					break;
				if(NS_LaunchTitle(progid, 0, procid) >= 0)
					break;
			}
			if(!loaded)
				format[0] = 0xF00FCACE; //invalidate
		}
		PatStay(0x00FF00); // Notif LED = Green
	}
	return;
}

u32 siz = 0x80;
u32 bsiz = 1;
u32 scrw = 2;
u32 bits = 8;
int scr = 0;

void netfuncOld3DS(void* __dummy_arg__)
{

#if DEBUG_VERBOSE==1
	osTickCounterStart(&tick_ctr_1);
	osTickCounterStart(&tick_ctr_2_dma);
#endif
	double timems_processframe = 0;
	double timems_writetosocbuf = 0;
	double timems_formatconvert = 0;

	u32 procid = 0;
	bool doDMA = true;

	PatStay(0x00FF00); // Notif LED = Green

	u8 dmaconf[0x18];
	populatedmaconf(dmaconf,0);

	PatPulse(0x7F007F); // Notif LED = Medium Purple
	threadrunning = 1;

	// Infinite loop unless it crashes or is halted by another application.
	while(threadrunning)
	{
        if(soc->avail())

        if(netfuncWaitForSettings() < 0)
        {
        	delete soc;
        	soc = nullptr;
        }

        if(!soc) break;

#if DEBUG_VERBOSE==1
        sendDebugFrametimeStats(timems_processframe,timems_writetosocbuf,&timems_dmaasync,timems_formatconvert);
#endif

        // If index 0 of the config block is non-zero (we are signaled by the PC to init)
        // And this ImportDisplayCaptureInfo function doesn't error...
        if(cfgblk[0] && GSPGPU_ImportDisplayCaptureInfo(&capin) >= 0)
        {
        	netfuncTestFramebuffer(&procid,&scr);

        	//tryMarioKartHotfix(&scrw);

            // Note: We control how often this loop runs
            // compared to how often the capture info is checked,
            // by changing the loopcnt variable. (Renamed to loopy, lol.)
            // By default, the ratio was 1:1
        	//
        	// If loopy = 2, the ratio is 2:1 (do this twice for every one time we test framebuffers)
        	//
        	// Increasing this would lead to a theoretical speed increase,
        	// but probably not noticeable in practice.
            for(int loopy = 2; loopy > 0; loopy--)
            {
                //soc->setPakSize(0);
            	tryStopDma();

                int imgsize = 0;

                switch(cfgblk[4])
                {
					case 0: // JPEG
						makeJpegImage(&timems_formatconvert, &timems_processframe, scr, &scrw, &bsiz, &imgsize);
						break;
					case 1: // Targa / TGA
						makeTargaImage(&timems_formatconvert, &timems_processframe, scr, &scrw, &bits, &imgsize);
						break;
					default:
						break; // This case shouldn't occur.
                }


				// Screen-chunk index ranges from 0 to 7 (Old-3DS only)
				u8 b = 0b00001000 + (offs[scr]);
				soc->setPakSubtypeB(b);
                // Current progress through one complete frame
                // (Only applicable to Old-3DS)
				offs[scr]++;
                if(offs[scr] >= limit[scr])
                	offs[scr] = 0;

                if(cfgblk[3] == 01) // Top Screen Only
                	scr = 0;
                else if(cfgblk[3] == 02) // Bottom Screen Only
                	scr = 1;
                else if(cfgblk[3] == 03) // Both Screens
                	scr = !scr;
                //else if(cfgblk[0] == 04)
                // Planning to add more complex functionality with prioritizing one
                // screen over the other, like NTR. Maybe.

                // TODO: This code will be redundant in the future, if not already.
                //
                // TODO: Does this even return a correct value? Even remotely?
                siz = (capin.screencapture[scr].framebuf_widthbytesize * stride[scr]); // Size of the entire frame (in bytes)
                bsiz = capin.screencapture[scr].framebuf_widthbytesize / 240; // Size of a single pixel in bytes (???)
                scrw = capin.screencapture[scr].framebuf_widthbytesize / bsiz; // Screen "Width" (Usually 240)
                bits = 4 << bsiz; // ?

                Handle prochand = 0;
                if(procid) if(svcOpenProcess(&prochand, procid) < 0) procid = 0;


                if(doDMA)
                {
                	u32 srcprochand = prochand ? prochand : 0xFFFF8001;
                	u8* srcaddr = (u8*)capin.screencapture[scr].framebuf0_vaddr + (siz * offs[scr]);

                	//u8 fm = format[scr] & 0b0111;
                	//u8 formatsbyte = (fm << 4) + fm; //0b01110111;

                	//u8 gputransferflag[4] = {0b00100000,formatsbyte,0,0};

#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_2_dma);
#endif

                	int r = svcStartInterProcessDma(&dmahand,0xFFFF8001,screenbuf,srcprochand,srcaddr,siz,dmaconf);

                	//int r = GX_DisplayTransfer((u32*)srcaddr,(240 << 16) + 400,(u32*)screenbuf,(240 << 16) + 400,*((u32*)gputransferflag));

                	if(r < 0)
                	{
                		procid = 0;
						format[scr] = 0xF00FCACE; //invalidate
                	}
                	else
                	{
#if DEBUG_VERBOSE==1
                		if(dmastatusthreadrunning == 0)
                		{
                			// Old-3DS specific configuration to optimize performance of everything *except* that thread, lol.
                			//
                			// Note: At lowest possible priority, results will be less consistent
                			// and on average less accurate. But it still produces usable results
                			// every once in a while, and this isn't a high-priority feature anyway.
                			threadCreate(waitforDMAtoFinish, nullptr, 0x80, 0x3F, 0, true);
                		}
#endif
                	}
                }

                if(prochand)
                {
                    svcCloseHandle(prochand);
                    prochand = 0;
                }

                // If size is 0, don't send the packet.
                if(soc->getPakSize())
                {
#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_1);
#endif

                	soc->wribuf();

#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_1);
					timems_writetosocbuf = osTickCounterRead(&tick_ctr_1);
#endif
                }

                // Limit this thread to do other things? (On Old-3DS)
                // TODO: Fine-tune Old-3DS performance.
                //
                // Note to self, removing this entirely will break things
                // (except maybe in extreme cases, like if priority equals 0x3F,
                //  but this remains untested for now...)
                svcSleepThread(5e6);
                // 5 x 10 ^ 6 nanoseconds (iirc)
            }
        }
        else yield();
    }
    // Notif LED = Flashing yellow and purple
    memset(&pat.r[0], 0xFF, 16);
    memset(&pat.g[0], 0xFF, 16);
    memset(&pat.b[0], 0x00, 16);
    memset(&pat.r[16],0x7F, 16);
    memset(&pat.g[16],0x00, 16);
    memset(&pat.b[16],0x7F, 16);
    pat.ani = 0x0406;
    PatApply();
    if(soc)
    {
        delete soc;
        soc = nullptr;
    }
    if(dmahand)
    {
        svcStopDma(dmahand);
        svcCloseHandle(dmahand);
    }
    threadrunning = 0;
}

void netfuncNew3DS(void* __dummy_arg__)
{
	osTickCounterStart(&tick_ctr_1);
	osTickCounterStart(&tick_ctr_2_dma);
	double timems_processframe = 0;
	double timems_writetosocbuf = 0;
	double timems_formatconvert = 0;
	u32 procid = 0;
	bool doDMA = true;

	//New3DS-Specific
	osSetSpeedupEnable(1);

	PatStay(0x00FF00); // Notif LED = Green

	u8 dmaconf[0x18];
	populatedmaconf(dmaconf,0);

	PatPulse(0x7F007F); // Notif LED = Medium Purple
	threadrunning = 1;

	// Infinite loop unless it crashes or is halted by another application.
	while(threadrunning)
	{
        if(soc->avail())

        if(netfuncWaitForSettings() < 0)
        {
        	delete soc;
        	soc = nullptr;
        }

        if(!soc) break;

#if DEBUG_VERBOSE==1
        sendDebugFrametimeStats(timems_processframe,timems_writetosocbuf,&timems_dmaasync,timems_formatconvert);
#endif

        // If index 0 of the config block is non-zero (we are signaled by the PC to init)
        // And this ImportDisplayCaptureInfo function doesn't error...
        if(cfgblk[0] && GSPGPU_ImportDisplayCaptureInfo(&capin) >= 0)
        {
        	netfuncTestFramebuffer(&procid,&scr);

        	//tryMarioKartHotfix(&scrw);

            // Note: We control how often this loop runs
            // compared to how often the capture info is checked,
            // by changing the loopcnt variable. (Renamed to loopy, lol.)
            // By default, the ratio was 1:1
        	//
        	// If loopy = 2, the ratio is 2:1 (do this twice for every one time we test framebuffers)
        	//
        	// Increasing this would lead to a theoretical speed increase,
        	// but probably not noticeable in practice.
            for(int loopy = 2; loopy > 0; loopy--)
            {
                //soc->setPakSize(0);
            	tryStopDma();

            	//New3DS-Specific
            	svcFlushProcessDataCache(0xFFFF8001, (u8*)screenbuf, capin.screencapture[scr].framebuf_widthbytesize * 400);

                int imgsize = 0;

                switch(cfgblk[4])
                {
					case 0: // JPEG
						makeJpegImage(&timems_formatconvert, &timems_processframe, scr, &scrw, &bsiz, &imgsize);
						break;
					case 1: // Targa / TGA
						makeTargaImage(&timems_formatconvert, &timems_processframe, scr, &scrw, &bits, &imgsize);
						break;
					default:
						break; // This case shouldn't occur.
                }

                if(cfgblk[3] == 01) // Top Screen Only
                	scr = 0;
                else if(cfgblk[3] == 02) // Bottom Screen Only
                	scr = 1;
                else if(cfgblk[3] == 03) // Both Screens
                	scr = !scr;
                //else if(cfgblk[0] == 04)
                // Planning to add more complex functionality with prioritizing one
                // screen over the other, like NTR. Maybe.

                // TODO: This code will be redundant in the future, if not already.
                //
                // TODO: Does this even return a correct value? Even remotely?
                siz = (capin.screencapture[scr].framebuf_widthbytesize * stride[scr]); // Size of the entire frame (in bytes)
                bsiz = capin.screencapture[scr].framebuf_widthbytesize / 240; // Size of a single pixel in bytes (???)
                scrw = capin.screencapture[scr].framebuf_widthbytesize / bsiz; // Screen "Width" (Usually 240)
                bits = 4 << bsiz; // ?


                Handle prochand = 0;
                if(procid) if(svcOpenProcess(&prochand, procid) < 0) procid = 0;


                if(doDMA)
                {
                	u32 srcprochand = prochand ? prochand : 0xFFFF8001;
                	u8* srcaddr = (u8*)capin.screencapture[scr].framebuf0_vaddr + (siz * offs[scr]);

                	//u8 fm = format[scr] & 0b0111;
                	//u8 formatsbyte = (fm << 4) + fm; //0b01110111;

                	//u8 gputransferflag[4] = {0b00100000,formatsbyte,0,0};

#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_2_dma);
#endif

                	int r = svcStartInterProcessDma(&dmahand,0xFFFF8001,screenbuf,srcprochand,srcaddr,siz,dmaconf);

                	//int r = GX_DisplayTransfer((u32*)srcaddr,(240 << 16) + 400,(u32*)screenbuf,(240 << 16) + 400,*((u32*)gputransferflag));

                	if(r < 0)
                	{
                		procid = 0;
						format[scr] = 0xF00FCACE; //invalidate
                	}
                	else
                	{
#if DEBUG_VERBOSE==1
                		if(dmastatusthreadrunning == 0)
                		{
                			// Old-3DS specific configuration to optimize performance of everything *except* that thread, lol.
                			//
                			// Note: At lowest possible priority, results will be less consistent
                			// and on average less accurate. But it still produces usable results
                			// every once in a while, and this isn't a high-priority feature anyway.
                			//
                			// On New-3DS, this will help very very slightly. Honestly worth it
                			threadCreate(waitforDMAtoFinish, nullptr, 0x80, 0x3F, 0, true);
                		}
#endif
                	}
                }

                if(prochand)
                {
                    svcCloseHandle(prochand);
                    prochand = 0;
                }

                // If size is 0, don't send the packet.
                if(soc->getPakSize())
                {
#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_1);
#endif

                	soc->wribuf();

#if DEBUG_VERBOSE==1
                	osTickCounterUpdate(&tick_ctr_1);
					timems_writetosocbuf = osTickCounterRead(&tick_ctr_1);
#endif
                }
            }
        }
        else yield();
    }
    // Notif LED = Flashing yellow and purple
    memset(&pat.r[0], 0xFF, 16);
    memset(&pat.g[0], 0xFF, 16);
    memset(&pat.b[0], 0x00, 16);
    memset(&pat.r[16],0x7F, 16);
    memset(&pat.g[16],0x00, 16);
    memset(&pat.b[16],0x7F, 16);
    pat.ani = 0x0406;
    PatApply();
    if(soc)
    {
        delete soc;
        soc = nullptr;
    }
    if(dmahand)
    {
        svcStopDma(dmahand);
        svcCloseHandle(dmahand);
    }
    threadrunning = 0;
}


static FILE* f = nullptr;

ssize_t stdout_write(struct _reent* r, void* fd, const char* ptr, size_t len)
{
    if(!f) return 0;
    fputs("[STDOUT] ", f);
    return fwrite(ptr, 1, len, f);
}

ssize_t stderr_write(struct _reent* r, void* fd, const char* ptr, size_t len)
{
    if(!f) return 0;
    fputs("[STDERR] ", f);
    return fwrite(ptr, 1, len, f);
}

static const devoptab_t devop_stdout = { "stdout", 0, nullptr, nullptr, stdout_write, nullptr, nullptr, nullptr };
static const devoptab_t devop_stderr = { "stderr", 0, nullptr, nullptr, stderr_write, nullptr, nullptr, nullptr };

int main()
{
    mcuInit();
    nsInit();
    
    // Isn't this already initialized to null?
    soc = nullptr;
    
#if DEBUG_BASIC==1
    f = fopen("HzLog.log", "a");
    if(f != NULL)
    {
        devoptab_list[STD_OUT] = &devop_stdout;
		devoptab_list[STD_ERR] = &devop_stderr;

		setvbuf(stdout, nullptr, _IONBF, 0);
		setvbuf(stderr, nullptr, _IONBF, 0);
    }
    printf("Hello World? Does this work? lol\n");
#endif
    
    memset(&pat, 0, sizeof(pat));
    memset(&capin, 0, sizeof(capin));
    memset(cfgblk, 0, sizeof(cfgblk));
    
    u32 soc_service_buf_siz = 0;
	u32 screenbuf_siz = 0;
	u32 bufsoc_siz = 0;
	u32 netfunc_thread_stack_siz = 0;
	int netfunc_thread_priority = 0;
	int netfunc_thread_cpu = 0;

    isold = APPMEMTYPE <= 5;
    if(isold) // is Old-3DS
    {

#if DEBUG_O3DSNEWINTERLACE==1
    	limit[0] = 1;
    	limit[1] = 1;
    	stride[0] = 400;
    	stride[1] = 320;

    	soc_service_buf_siz = 0x20000;
    	screenbuf_siz = 400 * 120 * 3;
    	bufsoc_siz = 0x10000;

    	netfunc_thread_stack_siz = 0x2000;

    	netfunc_thread_priority = 0x21;
    	netfunc_thread_cpu = 1;
#else
    	limit[0] = 8; // Capture the screen in 8 chunks
    	limit[1] = 8;
    	stride[0] = 50; // Screen / Framebuffer width (divided by 8)
    	stride[1] = 40;

    	soc_service_buf_siz = 0x10000;
    	screenbuf_siz = 50 * 240 * 4;
    	bufsoc_siz = 0xC000;

    	netfunc_thread_stack_siz = 0x2000;

    	// Original values; priority = 0x21, CPU = 1
    	//
    	netfunc_thread_priority = 0x21;
    	netfunc_thread_cpu = 1;
#endif

    }
    else // is New-3DS (or New-2DS)
    {
    	limit[0] = 1;
    	limit[1] = 1;
    	stride[0] = 400;
    	stride[1] = 320;

    	soc_service_buf_siz = 0x200000;
    	screenbuf_siz = 400 * 240 * 4;
    	bufsoc_siz = 0x70000;

    	netfunc_thread_stack_siz = 0x4000;

    	// Original values; priority = 0x08, CPU = 3
    	//
		// Setting priority around 0x10 (16) makes it stop slowing down Home Menu and games.
        // Priority:
        // Range from 0x00 to 0x3F. Lower numbers mean higher priority.
		netfunc_thread_priority = 0x10;
		// Processor ID:
		// -2 = Default (Don't bother using this)
		// -1 = All CPU cores(?)
		// 0 = Appcore and 1 = Syscore on Old-3DS
		// 2 and 3 are allowed on New-3DS (for Base processes)
		//
		netfunc_thread_cpu = 2;
    }
    
    PatStay(0x0000FF); // Notif LED = Red
    
    // Initialize AC service, for Wifi stuff.
    acInit();
    
    // TODO: Try experimenting with a hacky way to write to this buffer even though we're not supposed to have access.
    // I know it'll break things. But I am still interested to see what'll happen, lol.
	ret = socInit((u32*)memalign(0x1000, soc_service_buf_siz), soc_service_buf_siz);
    if(ret < 0) *(u32*)0x1000F0 = ret;
    
    jencode = tjInitCompress();
    if(!jencode) *(u32*)0x1000F0 = 0xDEADDEAD;
    
    // Initialize communication with the GSP service, for GPU stuff
    gspInit();
    
	screenbuf = (u32*)memalign(8, screenbuf_siz);
    
    // If memalign returns null or 0
    if(!screenbuf)
    {
        makerave();
        svcSleepThread(2e9);
        hangmacro();
    }
    
    
    if((__excno = setjmp(__exc))) goto killswitch;
      
#ifdef _3DS
    std::set_unexpected(CPPCrashHandler);
    std::set_terminate(CPPCrashHandler);
#endif
    
    netreset:
    
    if(sock)
    {
        close(sock);
        sock = 0;
    }
    
    // at boot, haznet is set to 0. so skip this on the first run through
    if(haznet && errno == EINVAL)
    {
        errno = 0;
        PatStay(0x00FFFF); // Notif LED = Yellow
        while(checkwifi()) yield();
    }
    
    if(checkwifi())
    {
    	int r;


#if DEBUG_USEUDP==1
		// UDP (May not work!)

		// For third argument, 0 is fine as there's only one form of datagram service(?)
		// But also, if IPPROTO_UDP is fine, I may stick with that.

		r = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#endif

#if DEBUG_USEUDP==0
		r = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // TCP (This works; don't change it.)
#endif

        if(r <= 0)
        {
#if DEBUG_BASIC==1
            printf("socket error: (%i) %s\n", errno, strerror(errno));
#endif
            hangmacro();
        }
        
        sock = r;
        
        struct sockaddr_in sao;
        sao.sin_family = AF_INET;
        sao.sin_addr.s_addr = gethostid();
        sao.sin_port = htons(port);
        
        if(bind(sock, (struct sockaddr*)&sao, sizeof(sao)) < 0)
        {
#if DEBUG_BASIC==1
            printf("bind error: (%i) %s\n", errno, strerror(errno));
#endif
            hangmacro();
        }
        
#if DEBUG_USEUDP==0// TCP-only code block
        {
			if(listen(sock, 1) < 0)
			{
#if DEBUG_BASIC==1
				printf("listen error: (%i) %s\n", errno, strerror(errno));
#endif
				hangmacro();
			}
        }
#endif
    }
    
    
    reloop:
    
    if(!isold) osSetSpeedupEnable(1);
    
    PatPulse(0xFF40FF);
    if(haznet) PatStay(0xCCFF00); // Notif LED = 100% Green, 75% Blue
    else PatStay(0x00FFFF); // Notif LED = Yellow
    
    while(1)
    {
        hidScanInput();
        kDown = hidKeysDown();
        kHeld = hidKeysHeld();
        kUp = hidKeysUp();
        
        // If any buttons are pressed, make the Notif LED pulse red
        // (Annoying and waste of CPU time. -C)
        if(kDown) PatPulse(0x0000FF);

        if(kHeld == (KEY_SELECT | KEY_START)) break;
        
        if(!soc)
        {
            if(!haznet)
            {
                if(checkwifi()) goto netreset;
            }
            else if(pollsock(sock, POLLIN) == POLLIN)
            {
            	// Client
                int cli = accept(sock, (struct sockaddr*)&sai, &sizeof_sai);
                if(cli < 0)
                {
#if DEBUG_BASIC==1
                    printf("Failed to accept client: (%i) %s\n", errno, strerror(errno));
#endif
                    if(errno == EINVAL) goto netreset;
                    PatPulse(0x0000FF); // Notif LED = Red
                }
                else
                {
                    PatPulse(0x00FF00); // Notif LED = Green
                    soc = new bufsoc(cli, bufsoc_siz);
                    k = soc->pack();
                    
                    if(isold)
                    {
                    	netthread = threadCreate(netfuncOld3DS, nullptr, netfunc_thread_stack_siz, netfunc_thread_priority, netfunc_thread_cpu, true);
                    }
                    else
                    {
                    	netthread = threadCreate(netfuncNew3DS, nullptr, netfunc_thread_stack_siz, netfunc_thread_priority, netfunc_thread_cpu, true);
                	}
                    
                    if(!netthread)
                    {
                    	// Notif LED = Blinking Red
                        memset(&pat, 0, sizeof(pat));
                        memset(&pat.r[0], 0xFF, 16);
                        pat.ani = 0x102;
                        PatApply();
                        
                        svcSleepThread(2e9);
                    }
                    
                    // Could above and below if statements be combined? lol -H
                    // No, we wait a little while to see if netthread is still
                    // not running or if it was just slow starting up. -C
                    
                    if(netthread)
                    {
                    	// After threadrunning = 1, we continue
                        while(!threadrunning) yield();
                    }
                    else
                    {
                        delete soc;
                        soc = nullptr;
                        hangmacro();
                    }
                }
            }
            else if(pollsock(sock, POLLERR) == POLLERR)
            {
#if DEBUG_BASIC==1
                printf("POLLERR (%i) %s", errno, strerror(errno));
#endif
                goto netreset;
            }
        }
        
        if(netthread && !threadrunning)
        {
            netthread = nullptr;
            goto reloop;
        }
        
        // VRAM Corruption function :)
        //if((kHeld & (KEY_ZL | KEY_ZR)) == (KEY_ZL | KEY_ZR))
        //{
        //    u32* ptr = (u32*)0x1F000000;
        //    int o = 0x00600000 >> 2;
        //    while(o--) *(ptr++) = rand();
        //}
        
        yield();
    }
    
    killswitch:
    
    PatStay(0xFF0000); // Notif LED = Blue
    
    if(netthread)
    {
        threadrunning = 0;
        
        volatile bufsoc** vsoc = (volatile bufsoc**)&soc;
        while(*vsoc) yield(); //pls don't optimize kthx
    }
    
    if(soc) delete soc;
    else close(sock);
    
#if DEBUG_BASIC==1
    puts("Shutting down sockets...");
#endif
    SOCU_ShutdownSockets();
    
    socExit();
    
    gspExit();
    
    acExit();
    
    if(f)
    {
        fflush(f);
        fclose(f);
    }
    
    PatStay(0);
    
    nsExit();
    
    mcuExit();
    
    // new code consideration
    // APT_PrepareToCloseApplication(false);

    return 0;
}
