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


// Debug Log Levels!
// Disable all for marginally better performance.
#define DEBUG_BASIC 1 //   Only log the most important and critical errors.
#define DEBUG_VERBOSE 0 // Log many more unnecessary details, including live performance profiling data sent to ChokiStream.

// Use experimental UDP instead of TCP. (Unfinished; doesn't work)
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
#include "service/hid.h"
#include "misc/pattern.h"
#include "misc/setdmacfg.h"
#include "imanip/imanip.h"
#include "imanip/colormap.h"
#include "netfunc/nfhelp.h"

#include "tga/targa.h"
#include <turbojpeg.h>
}

#include <exception>

#include "utils.hpp"

extern u32 kDown;
extern u32 kHeld;
//extern u32 kUp;

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
        hidScanInputDirectIO();\
        if(kHeld == (KEY_SELECT | KEY_START))\
        {\
            goto killswitch;\
        }\
        yield();\
    }\
        }

int checkwifi();
int pollsock(int,int,int);
void CPPCrashHandler();

inline int setCpuResourceLimit(u32); // Unfinished, doesn't work.
void waitforDMAtoFinish(void*);
void debugPrint(u8, const char*);
void sendDebugFrametimeStats(double,double,double*,double);
int allocateScreenbufMem(u32**);

// netfunc
void newThreadMainFunction(void*);

// Helper functions for netfunc
inline int netfuncWaitForSettings();
void makeTargaImage(double* timems_fc, double* timems_pf, int scr, u32 scrw, u32 scrh, int offsY, int offsX, u32* bits, int* imgsize, u32* myformat, bool isInterlaced, bool interlacedRowSwitch, bool makeColorMappedTga);
void makeJpegImage(double*,double*,int,u32*,u32*,int*,u32*,bool,bool);
void netfuncTestFramebuffer(u32*, GSPGPU_CaptureInfo, GSPGPU_CaptureInfo);


int main(); // So you can call main from main (:



static int haznet = 0;
int checkwifi()
{
    haznet = 0;
    u32 wifi = 0;
    hidScanInputDirectIO();
    if(kHeld == (KEY_SELECT | KEY_START)) return 0;
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

    u32 getPakSize()
    {
        return *( (u32*)(bufferptr+4) );
    }

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
        //puts("attempting recv function call...");
        int ret = recv(socketid, bufferptr, 8, flags);
        //printf("incoming packet type = %i\nsubtype1 = %i\nsubtype2 = %i\nrecv function return value = %i\n",bufferptr[0],bufferptr[1],bufferptr[2],ret);

        if(ret < 0) return -errno;
        if(ret < 8) return -1;

        u32 reads_remaining = getPakSize();
        //printf("incoming packet size = %i\nrecv return value = %i\n",reads_remaining,ret);

        // Copy data to the buffer

        u32 offs = bufsoc_pak_data_offset;
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
        u32 mustwri = getPakSize() + bufsoc_pak_data_offset;
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
        int len = 0;

        va_list args;
        va_start(args, c);

        // Is this line of code broken? I give up
        len = vsprintf((char*)(bufferptr+bufsoc_pak_data_offset), c, args);

        va_end(args);

        if(len < 0)
        {
            puts("out of memory");
            return -1;
        }

        //printf("Packet error %i: %s\n", p->packettype, p->data + 1);
        setPakType(0xFF);
        setPakSubtype(0x00);
        setPakSize(len);

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
    PatPulse(0x0000FF);

    svcSleepThread(1e9);

    hangmacro();

    killswitch:
    longjmp(__exc, 1);
}


extern "C" u32 __get_bytes_per_pixel(GSPGPU_FramebufferFormats format);

const int port = 6464;

// Whether or not this is running on an 'Old' 3DS
static int isold = 1;

static Result ret = 0;

// Related to screen capture. Dimensions and color format.
static u32 offs[2] = {0, 0};
static u32 limit[2] = {1, 1};
static u32 stride[2] = {80, 80};
static u32 format[2] = {0xF00FCACE, 0xF00FCACE};

// Config Block
static u8 cfgblk[0x100];

static int sock = 0;

static struct sockaddr_in sai;
static socklen_t sizeof_sai = sizeof(sai);

static bufsoc* soc = nullptr;

static bufsoc::packet* k = nullptr;

static Thread netthread = 0;
static vu32 threadrunning = 0;

static u32* screenbuf = nullptr;

static tga_image img;
static tjhandle jencode = nullptr;

static TickCounter tick_ctr_fps;
static TickCounter tick_ctr_1;
static TickCounter tick_ctr_2_dma;

// This only applies to Interlaced mode.
// false = odd rows (1-239, DMA start position is normal)
// true  = even rows (2-240, DMA start position is offset)
static bool interlacedRowSwitch = false;

static bool isStoredFrameInterlaced = false;
static bool isDmaSetForInterlaced = false;

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

//void debugPrintRemote(const char *debug_string)
void debugPrintRemote(char* c, ...)
{
    int len = 0;

    va_list args;
    va_start(args, c);

    len = vsprintf((char*)(soc->bufferptr+bufsoc_pak_data_offset), c, args);

    va_end(args);

    if(len < 0)
    {
        puts("debugPrintRemote: out of memory?");
        return;
    }

    soc->setPakType(0xFF);
    soc->setPakSubtype(03);
    soc->setPakSize(len);

    soc->wribuf();
    return;
}

// Essentially puts() with an extra step.
void debugPrint(u8 log_level, const char *debug_string)
{
    if(log_level == 0)
    {
        puts(debug_string);
        // TODO: If debug is disabled and SD Card can't be accessed, send this message to ChokiStream.
    }

#if DEBUG_BASIC==1
    else if(log_level == 1)
    {
        puts(debug_string);
    }
#endif

#if DEBUG_VERBOSE==1
    else if(log_level == 2)
    {
        puts(debug_string);
    }
#endif

    return;
}

void sendDebugFrametimeStats(double ms_compress, double ms_writesocbuf, double* ms_dma, double ms_convert)
{
    const u32 charbuflimit = 100 + sizeof(char);
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


// Returns -1 on an error, and expects the calling function to close the socket.
// Returns 1 on success.
int netfuncWaitForSettings()
{
    while(1)
    {
        if((kHeld & (KEY_SELECT | KEY_START)) == (KEY_SELECT | KEY_START))
            return -1;

        debugPrint(1, "Reading incoming packet...");

        int r = soc->readbuf();
        if(r <= 0)
        {
            debugPrint(1, "Failed to recvbuf...");
            debugPrint(1, strerror(errno));
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
            case 0x02: // Init (New ChirunoMod Packet Specification)
                cfgblk[0] = 1;
                return 1;

            case 0x03: // Disconnect (new packet spec)
                cfgblk[0] = 0;
                debugPrint(1, "Received packet type $03, forcibly disconnecting...");
                return -1;

            case 0x04: // Settings input (new packet spec)

                switch(i)
                {
                case 1: // JPEG Quality (1-100%)
                case 8:
                    if(j > 100)
                        j = 100;
                    else if(j < 1)
                        j = 1;
                    cfgblk[i] = j;
                    return 1;

                case 2: // CPU Cap value / CPU Limit / App Resource Limit

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

                case 3: // Which Screen(s)
                    if(j != 0 && j < 4)
                        cfgblk[3] = j;
                    return 1;

                case 4: // Image Format (JPEG or TGA?)
                case 9:
                    if(j > 1)
                        j = 0;
                    cfgblk[i] = j;
                    return 1;

                case 5: // Request to use Interlacing (yes or no)
                    j = j?1:0;

                    if(isold)
                    {
                        /* interlacing is disabled on o3DS
                        // If o3DS toggles Interlaced, signal to re-allocate framebuffer.
                        if(j != cfgblk[5])
                        {
                            cfgblk[5] = j;
                            return 9;
                        }
                        */
                    }
                    else
                    {
                        cfgblk[5] = j;
                    }
                    return 1;

                case 6: // priority screen
                    if(j > 1)
                        j = 0;
                    cfgblk[i] = j;
                    return 1;

                case 7: // priority factor
                    cfgblk[i] = j;
                    return 1;

                case 10: // use separate quality settings per screen?
                    cfgblk[i] = j?1:0;
                    return 1;

                case 11: // Special capture behavior for GB Virtual Console
                    if(j == 2) // force-enable
                        cfgblk[i] = 2;
                    else
                        cfgblk[i] = 0;
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
                    debugPrint(1, "Invalid packet ID...");
#if DEBUG_BASIC==1
                    printf("%i", soc->getPakType());
#endif
                    return -1;
            }

            return 1;
        }
    }

    return 1;
}

int allocateScreenbufMem(u32** myscreenbuf)
{
    u32 screenbuf_siz = 0;
    debugPrint(1, "(re)allocating memory for screenbuf...");
    if(isold) // is Old-3DS
    {
        /* interlacing is disabled on o3DS
        // If Interlaced
        if(cfgblk[5] == 1)
        {
            limit[0] = 1;
            limit[1] = 1;
            stride[0] = 400;
            stride[1] = 320;
            screenbuf_siz = 400 * 120 * 3;
        }
        else
        {
        */
        limit[0] = 8; // Capture the screen in 8 chunks
        limit[1] = 8;
        stride[0] = 50; // Screen / Framebuffer width (divided by 8)
        stride[1] = 40;
        screenbuf_siz = 50 * 240 * 3;
    }
    else
    {
        limit[0] = 1;
        limit[1] = 1;
        stride[0] = 400;
        stride[1] = 320;
        screenbuf_siz = 400 * 240 * 3;
    }

    int i = 0;
    while(i < 5 && (!*myscreenbuf) )
    {
        *myscreenbuf = (u32*)memalign(8, screenbuf_siz);

        if(!*myscreenbuf)
        {
            debugPrint(1, "memalign failed, retrying...");
            makerave();
            svcSleepThread(2e9);
        }
        i++;
    }

    if(!*myscreenbuf)
    {
        debugPrint(1, "Error: out of memory (memalign failed trying to allocate memory for screenbuf)");
        return -1;
    }
    else
    {
        return 0;
    }
}

void makeTargaImage(double* timems_fc, double* timems_pf, int scr, u32 scrw, u32 scrh, int offsY, int offsX, u32* bits, int* imgsize, u32* myformat, bool isInterlaced, bool interlacedRowSwitch, bool makeColorMappedTga)
{
#if DEBUG_VERBOSE==1
    *timems_fc = 0;
    osTickCounterUpdate(&tick_ctr_1);
#endif

    u32 newbits = *bits;

    switch(myformat[scr] & 0b111)
    {
    case 0: // RGBA8
        newbits = 32;
        break;

    case 1: // RGB8
        newbits = 24;
        break;

    case 2: // RGB565
        newbits = 17;
        break;

    case 3: // RGB5A1
        newbits = 16;
        break;

    case 4: // RGBA4
        newbits = 18;
        break;

    default:
        // Invalid
        break;
    }

    init_tga_image(&img, (u8*)screenbuf, scrw, scrh, newbits);

    img.image_type = TGA_IMAGE_TYPE_BGR_RLE;
    img.origin_y = offsY;
    img.origin_x = offsX;

    if(makeColorMappedTga)
    {
        void* color_map_data = nullptr;
        u16 color_map_length;
        u32 bufferMaxSize = 3 * 240 * (isold?50:400); // todo: put this elsewhere
        int r = convertRawToColorMappedTga((void*)screenbuf, bufferMaxSize, scrw*scrh, &color_map_data, &color_map_length);
        if(r >= 0)
        {
            img.image_type = TGA_IMAGE_TYPE_COLORMAP_RLE;
            img.color_map_type = 1;
            img.color_map_data = (uint8_t*)color_map_data;
            img.color_map_origin = 0;
            img.color_map_length = color_map_length;
            img.color_map_depth = img.pixel_depth;
            img.pixel_depth = 8;
        }
        else
        {
            printf("convertRawToColorMappedTga() returned -1\n");
        }
    }

    tga_result r = tga_write_to_FILE((soc->bufferptr+bufsoc_pak_data_offset), &img, imgsize);

#if DEBUG_VERBOSE==1
    osTickCounterUpdate(&tick_ctr_1);
    *timems_pf = osTickCounterRead(&tick_ctr_1);
#endif

    if(r != TGA_NOERR)
    {
        printf("tga_write_to_FILE() returned err %i\n", r);
    }
    else
    {
        u8 subtype_aka_flags = 0b00001000 + (scr * 0b00010000) + (format[scr] & 0b111);
        if(isInterlaced)
            subtype_aka_flags += 0b00100000 + (interlacedRowSwitch?0:0b01000000);

        soc->setPakType(01);
        soc->setPakSubtype(subtype_aka_flags);
        soc->setPakSize(*imgsize);
    }

    return;
}

void makeJpegImage(double* timems_fc, double* timems_pf, int scr, u32* scrw, u32* bsiz, int* imgsize, u32* myformat, bool isinterlaced, bool interlacedRowSwitch)
{
    u8 nativepixelformat = myformat[scr] & 0b111;
    u8 subtype_aka_flags = 0b00000000 + (scr * 0b00010000) + nativepixelformat;
    int tjpixelformat = 0;

    if(isinterlaced)
    {
        subtype_aka_flags += 0b00100000 + (interlacedRowSwitch?0:0b01000000);
    }

#if DEBUG_VERBOSE==1
    osTickCounterUpdate(&tick_ctr_1);
#endif

    switch(nativepixelformat)
    {
    case 0: // RGBA8
        //tjpixelformat = TJPF_RGBX;
        tjpixelformat = TJPF_BGR;
        *bsiz = 3;
        break;

    case 1: // RGB8
        tjpixelformat = TJPF_BGR;
        *bsiz = 3;
        break;

    case 2: // RGB565
        convert16to24_rgb565(stride[scr], *scrw, (u8*)screenbuf);
        tjpixelformat = TJPF_BGR;
        *bsiz = 3;
        break;

    case 3: // RGB5A1
        convert16to24_rgb5a1(stride[scr], *scrw, (u8*)screenbuf);
        tjpixelformat = TJPF_BGR;
        *bsiz = 3;
        break;

    case 4: // RGBA4
        convert16to24_rgba4(stride[scr], *scrw, (u8*)screenbuf);
        tjpixelformat = TJPF_BGR;
        *bsiz = 3;
        break;

    default:
        tjpixelformat = TJPF_BGR;
        //*bsiz = 3;
        break;
    }

    u8 qual;
    if(cfgblk[10])
        qual = cfgblk[scr?8:1];
    else
        qual = cfgblk[1];

#if DEBUG_VERBOSE==1
    osTickCounterUpdate(&tick_ctr_1);
    *timems_fc = osTickCounterRead(&tick_ctr_1);
#endif

    u8* destaddr = soc->bufferptr + bufsoc_pak_data_offset;

    if(!tjCompress2(jencode, (u8*)screenbuf, *scrw, (*bsiz) * (*scrw), stride[scr], tjpixelformat, &destaddr, (u32*)imgsize, TJSAMP_420, qual, TJFLAG_NOREALLOC | TJFLAG_FASTDCT))
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

u8 getFormatBpp(u32 my_format)
{
    switch(my_format & 0b111)
    {
    case 0: // 32bpp
        return 32;

    case 1: // 24bpp
        return 24;

    default: // 16bpp
        return 16;
    }
}

/**
 * Analyze information for hints that the procId we are using has just become outdated.
 *
 * Currently, this is done by comparing whether the Capture Info has changed.
 * That's a pretty good indicator, especially the framebuf0_vaddr and framebuf1_vaddr.
 * Though it returns a lot of false-positives, probably.
 * And this strategy isn't perfect anyway.
 */
bool outdatedProcIdHint(u32* procid, GSPGPU_CaptureInfo newCapInfo, GSPGPU_CaptureInfo oldCapInfo)
{
    bool isCaptureInfoChanged = false;

    // GSPGPU_CaptureInfo struct size is a multiple of 4, so this is fine.
    for(u32 s = 0; s < (sizeof(GSPGPU_CaptureInfo)/4); s++)
    {
        if(*(u32*)&newCapInfo != *(u32*)&oldCapInfo)
        {
            isCaptureInfoChanged = true;
            break;
        }
    }

    return isCaptureInfoChanged;
}

/**
 * Get procId of the current process in the foreground.
 * (The process which the current framebuffers belong to.)
 *
 * Note: This isn't necessary if the framebuffer is in VRAM,
 * as we already have read-access to that.
 *
 * Currently, the way of doing this is pretty hacky (but still Luma3DS-compatible).
 * And I don't think it works in all cases.
 * It relies on weird assumptions about AppID that I don't fully understand.
 * See: https://www.3dbrew.org/wiki/NS_and_APT_Services#AppIDs
 * TODO: Is there a better way of doing this?
 *
 * also TODO: potential bug...
 * The way we do things right now, we need a procId to DMA from the top framebuffer,
 * and we need a procId to DMA from the bottom framebuffer. With the way this code currently works,
 * we assume that those two needed procIds are the same.
 * Is it ever the case that they are different? Because if so, that will cause issues.
 * Specifically, DMA requests for one of the two screens will just fail.
 * That might be what's happening in Super Smash Bros, but I haven't looked into it.
 */
void getNewProcId(u32* procid, GSPGPU_CaptureInfo new_captureinfo)
{
    *procid = 0;

    //test for VRAM
    if( (u32)(new_captureinfo.screencapture[0].framebuf0_vaddr) < 0x1F600000 )
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
        u8 mediatype = 0;

        while(1)
        {
            // loaded = Registration Status(?) of the specified application.
            loaded = false;
            while(1)
            {
                NS_APPID appid;
                if(APT_GetAppletManInfo(APTPOS_NONE, nullptr, nullptr, nullptr, &appid) < 0)
                    break;
                appid = (NS_APPID)(appid & 0xFFFF);
                if(APT_GetAppletInfo(appid, &progid, &mediatype, &loaded, nullptr, nullptr) < 0)
                    break;
                if(loaded)
                    break;
                svcSleepThread(15e6);
            }
            if(!loaded)
                break;
            // see: https://github.com/ChainSwordCS/ChirunoMod/issues/11
            //if(mediatype == 2)
            //    progid = 0; // Game Card
            if(NS_LaunchTitle(progid, 0, procid) >= 0)
                break;
        }
        if(!loaded)
            format[0] = 0xF00FCACE; //invalidate
        PatStay(0x00FF00); // Notif LED = Green
    }
    return;
}

// "netfunc" v2.1
void newThreadMainFunction(void* __dummy_arg__)
{
#if DEBUG_VERBOSE==1
    osTickCounterStart(&tick_ctr_1);
    osTickCounterStart(&tick_ctr_2_dma);
#endif
    osTickCounterStart(&tick_ctr_fps);
    double timems_processframe = 0;
    double timems_writetosocbuf = 0;
    double timems_formatconvert = 0;
    double timems_frame = 0;

    u32 siz = 0x80;
    u32 bsiz = 1;
    u32 scrw = 2;
    u32 bits = 8;
    int scr = 0;
    u32 procid = 0;
    bool doDMA = true;

    u8 dma_cfg_0[0x18];
    u8 dma_cfg_1[0x18];
    u8* dma_config[2];
    dma_config[0] = dma_cfg_0;
    dma_config[1] = dma_cfg_1;
    initCustomDmaCfg(dma_config[0]);
    initCustomDmaCfg(dma_config[1]);

    threadrunning = 1;

    if(isold == 0){
        osSetSpeedupEnable(1);
    }

    PatStay(0x00FF00); // Notif LED = Green

    // Wait for the signal to start capturing and streaming frames.
    while(cfgblk[0] == 0)
    {
        if(soc->avail())
        {
            int ret_nwfs = netfuncWaitForSettings();
            if(ret_nwfs < 0)
            {
                delete soc;
                soc = nullptr;
            }
            else if(ret == 9)
            {
                debugPrint(1, "o3DS toggled Interlaced setting, reallocating screenbuf...");
                free(screenbuf);
                screenbuf = nullptr;
                yield(); // does this help
                allocateScreenbufMem(&screenbuf);
            }
        }
        if(!soc) break;
    }

    GSPGPU_CaptureInfo capInfo[2];
    memset(&capInfo[0], 0, sizeof(GSPGPU_CaptureInfo));
    memset(&capInfo[1], 0, sizeof(GSPGPU_CaptureInfo));
    // indicates which capInfo buffer was last updated.
    bool c = (bool)0;

    // fallback values
    format[0] = capInfo[(u8)c].screencapture[0].format & 0b111;
    format[1] = capInfo[(u8)c].screencapture[1].format & 0b111;

    // interlacing is disabled on o3DS and 24bpp frames
    if(cfgblk[5] && !isold && (format[scr] != 1))
        isDmaSetForInterlaced = true;
    else
        isDmaSetForInterlaced = false;

    //updateDmaCfg(dma_config[0], getFormatBpp(format[0]), cfgblk[5], capInfo[(u8)c].screencapture[0].framebuf_widthbytesize, scrw);
    //updateDmaCfg(dma_config[1], getFormatBpp(format[1]), cfgblk[5], capInfo[(u8)c].screencapture[1].framebuf_widthbytesize, scrw);

    u8 mainLoopCount = 0;
    u8 frameCount = 0;
    u8 priorityScreenFrames = 0;
    bool gbvcmode = false;
    bool gbvcmode_queuefulltopscreen = false;
    u8 imgfmt = 0;

    osTickCounterUpdate(&tick_ctr_fps);
    // thread main loop
    while(threadrunning)
    {
        if(!soc) break;

        PatStay(0x00FF00); // Notif LED = Green

        while(soc->avail())
        { // ?

            int ret_nwfs = netfuncWaitForSettings();
            if(ret_nwfs < 0)
            {
                delete soc;
                soc = nullptr;
                break;
            }
            /* interlacing is disabled on o3DS
            else if(ret_nwfs == 9)
            {
                debugPrint(1, "o3DS toggled Interlaced setting, reallocating screenbuf...");
                free(screenbuf);
                screenbuf = nullptr;
                yield(); // does this help
                allocateScreenbufMem(&screenbuf);
            }
            */
        }
        if(!soc) break;

#if DEBUG_VERBOSE==1
        sendDebugFrametimeStats(timems_processframe,timems_writetosocbuf,&timems_dmaasync,timems_formatconvert);
#endif

        // todo: this is hard-coded at the moment
        osTickCounterUpdate(&tick_ctr_fps);
        timems_frame = osTickCounterRead(&tick_ctr_fps);
        const double minTimePerFrame = 30.0; // at 30 FPS, 1 frame is 33.3 ms
        if(timems_frame > minTimePerFrame)
        {
            double frameLimitWait = minTimePerFrame - timems_frame;
            // note: svcSleepThread isn't exact in how long it waits. i forget the details.
            svcSleepThread(1e6 * frameLimitWait); // milliseconds to nanoseconds
            osTickCounterUpdate(&tick_ctr_fps);
        }

        if(GSPGPU_ImportDisplayCaptureInfo(&capInfo[(u8)!c]) < 0)
        {
            yield();
        }
        else
        {
            /**
             * capInfo[(u8)c] is up to date
             * capInfo[(u8)!c] is outdated
             */
            c = !c;

            if(outdatedProcIdHint(&procid,capInfo[(u8)c],capInfo[(u8)!c]))
            {
                tryStopDma(&dmahand);
                getNewProcId(&procid, capInfo[(u8)c]);
            }

            format[0] = capInfo[(u8)c].screencapture[0].format & 0b111;
            format[1] = capInfo[(u8)c].screencapture[1].format & 0b111;

            if(isold == 0){
                svcFlushProcessDataCache(0xFFFF8001, (u8*)screenbuf, capInfo[(u8)c].screencapture[scr].framebuf_widthbytesize * 400);
            }

            // interlacing is disabled on o3DS and 24bpp frames
            if(cfgblk[5] && !isold && (getFormatBpp(format[scr]) != 24))
                isDmaSetForInterlaced = true;
            else
                isDmaSetForInterlaced = false;



            /**
             * If we're outrunning DMA, stall for a bit and then svcStopDma.
             * Really we could wait indefinitely, but I'd prefer to have a sorta
             * timeout system in place, as I do here.
             */
            if(dmahand)
            {
                u32 dmaState = 0xF00FCACE;
                svcGetDmaState(&dmaState, dmahand);
                for(int i = 0; i < 420; i++)
                {
                    if((dmaState&0xFF) != 3)
                        break;
                    svcSleepThread(5e4); // Going higher (5e6, for example) may result in crashes.
                    svcGetDmaState(&dmaState, dmahand);
                }
                svcStopDma(dmahand);
                svcCloseHandle(dmahand);
                dmahand = 0;

#if DEBUG_BASIC==1
                if((dmaState&0xFF) != 4 && (dmaState&0xFF) != 3)
                    debugPrintRemote("Warning: dmaState=$%.2X (ret=%.8X)", (u8)(dmaState&0xFF), dmaState);
                    //printf("Warning: dmaState=$%.2X (ret=%.8X)", (u8)(dmaState&0xFF), dmaState);
#endif
            }

            int imgsize = 0;

            // interlaced
            if(isStoredFrameInterlaced)
            {
                scrw = scrw / 2;
            }

            bool makeColorMappedTga = false;
            u32 scrh;
            int offsY;
            int offsX;
            if(gbvcmode && scr == 0 && !gbvcmode_queuefulltopscreen)
            {
                makeColorMappedTga = true;
                scrh = 160;
                offsY = 120;
                offsX = 48;
            }
            else
            {
                scrh = stride[scr];
                offsY = (scr * 400) + (stride[scr] * offs[scr]);
                offsX = 0;
            }

            switch(imgfmt)
            {
            case 0:
                makeJpegImage(&timems_formatconvert, &timems_processframe, scr, &scrw, &bsiz, &imgsize, format, isStoredFrameInterlaced, interlacedRowSwitch);
                break;
            case 1:
                makeTargaImage(&timems_formatconvert, &timems_processframe, scr, scrw, scrh, offsY, offsX, &bits, &imgsize, format, isStoredFrameInterlaced, interlacedRowSwitch, makeColorMappedTga);
                break;
            default:
                break; // This case shouldn't occur.
            }

            //printf("height (scrw) = %i\n", scrw);
            //printf("screen size in bytes = %i\n", bsiz);
            //printf("interlace_px_offset = %i\n", interlace_px_offset);

            // increment screen fraction / part on o3DS
            if(isold){
                u8 b = 0b00001000 + (offs[scr]);
                soc->setPakSubtypeB(b);
                offs[scr]++;
                if(offs[scr] >= limit[scr])
                    offs[scr] = 0;
            }

            // screen switch
            if(interlacedRowSwitch == false || cfgblk[5] == 0)
            {
                frameCount++;
                if(cfgblk[3] == 3) // Both Screens
                {
                    if(scr == cfgblk[6])
                    {
                        priorityScreenFrames++;
                        if(priorityScreenFrames >= cfgblk[7])
                        {
                            priorityScreenFrames = 0;
                            scr = !scr;
                        }
                    }
                    else
                    {
                        scr = !scr;
                    }
                }
                else if(cfgblk[3] == 1) // Top Screen Only
                    scr = 0;
                else if(cfgblk[3] == 2) // Bottom Screen Only
                    scr = 1;
            }

            /**
             * Activate special GBVC mode
             * Slightly smart; only active when top screen is RGB5A1.
             * This is also to sidestep weird DMA-related crashing issues in RGB8 cases.
             */
            if((cfgblk[11] == 2) && (format[0] == 3))
            {
                gbvcmode = true;
                if((frameCount%128)-1 == 0)
                    gbvcmode_queuefulltopscreen = true;
                else
                    gbvcmode_queuefulltopscreen = false;
            }
            else
            {
                gbvcmode = false;
                gbvcmode_queuefulltopscreen = false;
            }

            siz = (capInfo[(u8)c].screencapture[scr].framebuf_widthbytesize * stride[scr]); // Size of the entire frame (in bytes)
            bsiz = capInfo[(u8)c].screencapture[scr].framebuf_widthbytesize / 240; // bytes per pixel (dumb)
            scrw = 240; // sure
            bits = 4 << bsiz; // bpp (dumb)

            Handle prochand = 0;
            if(procid) if(svcOpenProcess(&prochand, procid) < 0) procid = 0;

            u32 srcprochand = prochand ? prochand : 0xFFFF8001;
            u8* srcaddr = (u8*)capInfo[(u8)c].screencapture[scr].framebuf0_vaddr + (siz * offs[scr]);

            u32 tempstride; // for "workaround for DMA siz bug"
            if(gbvcmode && scr == 0 && !gbvcmode_queuefulltopscreen)
            {
                imgfmt = 1;
                scrw = 144;
                //siz = (getFormatBpp(format[scr]) / 8) * 160 * 144;
                siz = 2 * 144 * 160;
                //srcaddr += (siz * 120) + (getFormatBpp(format[scr])/8) * 48;
                srcaddr += capInfo[(u8)c].screencapture[scr].framebuf_widthbytesize * 120 + (getFormatBpp(format[scr])/8)*48;
                tempstride = 160;

                updateDmaCfg(dma_config[0], getFormatBpp(format[0]), isDmaSetForInterlaced?1:0, capInfo[(u8)c].screencapture[0].framebuf_widthbytesize, 144);
                updateDmaCfg(dma_config[1], getFormatBpp(format[1]), isDmaSetForInterlaced?1:0, capInfo[(u8)c].screencapture[1].framebuf_widthbytesize, 240);
            }
            else
            {
                if(cfgblk[10])
                    imgfmt = cfgblk[scr?9:4];
                else
                    imgfmt = cfgblk[4];
                siz = (getFormatBpp(format[scr]) / 8) * scrw * stride[scr];
                tempstride = stride[scr];

                updateDmaCfg(dma_config[0], getFormatBpp(format[0]), isDmaSetForInterlaced?1:0, capInfo[(u8)c].screencapture[0].framebuf_widthbytesize, scrw);
                updateDmaCfg(dma_config[1], getFormatBpp(format[1]), isDmaSetForInterlaced?1:0, capInfo[(u8)c].screencapture[1].framebuf_widthbytesize, scrw);
            }

#if DEBUG_VERBOSE==1
            osTickCounterUpdate(&tick_ctr_2_dma);
#endif

            if(isDmaSetForInterlaced)
            {
                isStoredFrameInterlaced = true;
                siz = siz / 2;
                interlacedRowSwitch = !interlacedRowSwitch;
                if(interlacedRowSwitch)
                    srcaddr += getFormatBpp(format[scr])/8;
            }
            else
            {
                isStoredFrameInterlaced = false;
            }

            // workaround for DMA Siz Bug (refer to docs)
            //siz += (getFormatBpp(format[scr])/8) * (16 * tempstride - 16);

            int ret_dma = svcStartInterProcessDma(&dmahand,0xFFFF8001,screenbuf,srcprochand,srcaddr,siz,dma_config[scr]);

            if(ret_dma < 0)
            {
                procid = 0;
                format[scr] = 0xF00FCACE; //invalidate
            }
            else
            {
#if DEBUG_VERBOSE==1
                if(dmastatusthreadrunning == 0)
                {
                    // Note: At lowest possible priority, results will be less consistent
                    // and on average less accurate. But it still produces usable results
                    // every once in a while, and this isn't a high-priority feature anyway.
                    threadCreate(waitforDMAtoFinish, nullptr, 0x80, 0x3F, 0, true);
                }
#endif
            }

            if(prochand)
            {
                svcCloseHandle(prochand);
                prochand = 0;
            }






            /**
             * send frame to client (while waiting for DMA)
             * (note: DMA doesn't actually take very long.)
             */
            if(!soc) break;
            if(soc->getPakSize()) // If size == 0, don't send
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

            // TODO: Fine-tune Old-3DS performance.
            if(isold){
                svcSleepThread(5e6);
                // 5 x 10 ^ 6 nanoseconds (iirc)
            }
        }
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
    hidExit(); // for some old versions of libctru

    mcuInit();
    nsInit();

    soc = nullptr;

    // cfgblk sane defaults
    cfgblk[1] = 70;
    cfgblk[3] = 1;
    cfgblk[6] = 0;
    cfgblk[7] = 1;
    cfgblk[8] = 70;
    cfgblk[9] = 1;

#if DEBUG_BASIC==1
    f = fopen("HzLog.log", "a");
    if((int)f <= 0) f = nullptr;
    else
    {
        devoptab_list[STD_OUT] = &devop_stdout;
        devoptab_list[STD_ERR] = &devop_stderr;

        setvbuf(stdout, nullptr, _IONBF, 0);
        setvbuf(stderr, nullptr, _IONBF, 0);
    }
    printf("ChirunoMod is starting...\n");
#endif

    memset(&pat, 0, sizeof(pat));
    memset(cfgblk, 0, sizeof(cfgblk));

    u32 soc_service_buf_siz = 0;
    u32 screenbuf_siz = 0;
    u32 bufsoc_siz = 0;
    u32 netfunc_thread_stack_siz = 0;
    int netfunc_thread_priority = 0;
    int netfunc_thread_cpu = 0;

    isold = APPMEMTYPE <= 5;

    PatStay(0x0000FF); // Notif LED = Red

    // Initialize AC service, for Wifi stuff.
    acInit();

    if(isold) // is Old-3DS
    {
        soc_service_buf_siz = 0x10000;
        bufsoc_siz = 0xC000; // Consider trying 0x10000, but that may do nothing but waste memory.
        netfunc_thread_stack_siz = 0x2000;
        netfunc_thread_priority = 0x14;
        netfunc_thread_cpu = 1;

        /* interlacing is disabled on o3DS
        // If Interlaced
        if(cfgblk[5] == 1)
        {
            limit[0] = 1;
            limit[1] = 1;
            stride[0] = 400;
            stride[1] = 320;
            screenbuf_siz = 400 * 120 * 3;
        }
        else
        {
        */
        limit[0] = 8; // Capture the screen in 8 chunks
        limit[1] = 8;
        stride[0] = 50; // Screen / Framebuffer width (divided by 8)
        stride[1] = 40;
        screenbuf_siz = 50 * 240 * 3;
        //}
    }
    else // is New-3DS (or New-2DS)
    {
        soc_service_buf_siz = 0x10000; //0x200000
        bufsoc_siz = 0x70000;
        netfunc_thread_stack_siz = 0x4000;

        limit[0] = 1;
        limit[1] = 1;
        stride[0] = 400;
        stride[1] = 320;
        screenbuf_siz = 400 * 240 * 3;

        // Original values; priority = 0x08, CPU = 3
        //
        // Setting priority around 0x10 (16) makes it stop slowing down Home Menu and games.
        // Priority:
        // Range from 0x00 to 0x3F. Lower numbers mean higher priority.
        netfunc_thread_priority = 0x10;
        //
        // Processor ID:
        // -2 = Default (Don't bother using this)
        // -1 = All CPU cores(?)
        // 0 = Appcore and 1 = Syscore on Old-3DS
        // 2 and 3 are allowed on New-3DS (for Base processes)
        netfunc_thread_cpu = 2;
    }


    // TODO: Try experimenting with a hacky way to write to this buffer even though we're not supposed to have access.
    // I know it'll break things. But I am still interested to see what'll happen, lol.
    ret = socInit((u32*)memalign(0x1000, soc_service_buf_siz), soc_service_buf_siz);
    if(ret < 0) *(u32*)0x1000F0 = ret;

    jencode = tjInitCompress();
    if(!jencode) *(u32*)0x1000F0 = 0xDEADDEAD;

    // Initialize communication with the GSP service, for GPU stuff
    gspInit();

    // screenbuf = (u32*)memalign(8, screenbuf_siz);

    // If memalign returns null or 0
    //if(!screenbuf)
    //{
    //    makerave();
    //    svcSleepThread(2e9);
    //    printf("crashed while trying to allocate memory for screenbuf\n");
    //    hangmacro();
    //}


    if((__excno = setjmp(__exc))) goto killswitch;

#ifdef _3DS
    std::set_unexpected(CPPCrashHandler);
    std::set_terminate(CPPCrashHandler);
#endif

    if( allocateScreenbufMem(&screenbuf) == -1)
    {
        hangmacro();
    }

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

    if(!isold) // is New-3DS
    {
        osSetSpeedupEnable(1);
    }

    if(haznet) PatStay(0xCCFF00); // Notif LED = 100% Green, 75% Blue
    else PatStay(0x00FFFF); // Notif LED = Yellow

    while(1)
    {
        hidScanInputDirectIO();

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
                }
                else
                {
                    soc = new bufsoc(cli, bufsoc_siz);
                    k = soc->pack();

                    netthread = threadCreate(newThreadMainFunction, nullptr, netfunc_thread_stack_siz, netfunc_thread_priority, netfunc_thread_cpu, true);

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
            if(haznet) PatStay(0xCCFF00); // Notif LED = 100% Green, 75% Blue
            else PatStay(0x00FFFF); // Notif LED = Yellow
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

    // APT_PrepareToCloseApplication(false);

    return 0;
}
