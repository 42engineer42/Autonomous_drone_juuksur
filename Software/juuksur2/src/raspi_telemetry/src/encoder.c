#include "encoder.h"

//#define VIDEO_WIDTH 336
//#define VIDEO_HEIGHT 244
#define VIDEO_WIDTH 336
#define VIDEO_HEIGHT 450
#define VIDEO_FPS 40

#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//#define TIME_START() start = clock();
//#define TIME_END(str) printf("%s %f\r\n", str, (float)(clock()-start)/CLOCKS_PER_SEC);

#define TIME_START()
#define TIME_END(x)

static clock_t start;

int pushFrame(RPIEncoder *e, int w, int h, int stride, char *data) {
    static int frame;
    int i, j;
    OMX_BUFFERHEADERTYPE *buf;
    OMX_BUFFERHEADERTYPE *out;
    OMX_ERRORTYPE err;
    TIME_START()
    buf = ilclient_get_input_buffer(e->video_encode, 200, 1);
    TIME_END("get_input_buffer")
    if(buf == NULL) {
        printf("No buffers from ilclient!\r\n");
    } else {
        TIME_START()
        OMX_VIDEO_PORTDEFINITIONTYPE *vid = &e->def.format.video;
        OMX_U32 filledLen;
#if 0
        char *y = buf->pBuffer;
        char *u = y + vid->nStride * vid->nSliceHeight;
        char *v = u + (vid->nStride >> 1) * (vid->nSliceHeight >> 1);
        // set y
        for(j = 0; j < vid->nFrameHeight; j++) {
            char *py = y + j * vid->nStride;
            char *oy = data + j*stride;
            for(i = 0; i < vid->nFrameWidth; i++) {
                py[0] = oy[0];
                oy ++;
                py++;
            }
        }
        // set u and v
        char *ou = data + vid->nFrameHeight*stride;
        char *ov = ou + (vid->nFrameHeight*stride)/4;
        for (j = 0; j < vid->nFrameHeight / 2; j++) {
            char *pu = u + j * (vid->nStride >> 1);
            char *pv = v + j * (vid->nStride >> 1);
            char *opu = ou + j * (stride>>1);
            char *opv = ov + j * (stride>>1);
            for (i = 0; i < vid->nFrameWidth / 2; i++) {
                pu[0] = opu[0];
                pv[0] = opv[0];
                opu ++;
                opv ++;
                pu++;
                pv++;
            }
        }
        /*for (j = 0; j < vid->nFrameHeight / 2; j++) {
            char *py = y + 2 * j * vid->nStride;
            char *pu = u + j * (vid->nStride >> 1);
            char *pv = v + j * (vid->nStride >> 1);

            char *oy = j*stride*2;
            // this corresponds to 4x4 area (only y is per pixel)
            for (i = 0; i < vid->nFrameWidth / 2; i++) {
                //int z = (((i + frame) >> 4) ^ ((j + frame) >> 4)) & 15;
                py[0] = py[1] = py[vid->nStride] = py[vid->nStride + 1] = 0x80 + z * 0x8;
                pu[0] = 0x00 + z * 0x10;
                pv[0] = 0x80 + z * 0x30;
                if(i < w && j < h) {
                    //uint8_t *oy = data + 
                    py[0] = oy
                    //py[0] = da
                } else {
                    pu[0] = 0;
                    py[0] = 0;
                    pv[0] = 0;
                }
                py += 2;
                pu++;
                pv++;

                oy+=2;
            }
        } */
        buf->nFilledLen = (vid->nStride * vid->nSliceHeight * 3) >> 1;
#else 
        char *b = buf->pBuffer;
        char *ib = data;
        for(j = 0; j < vid->nFrameHeight; j++) {
            b = buf->pBuffer + j*vid->nStride*3;
            ib = data + j*stride;
            for(i = 0; i < vid->nFrameWidth; i++) {
                b[0] = ib[2];
                b[1] = ib[1];
                b[2] = ib[0];
                b+=3;
                ib+=3;
            }
        }
        buf->nFilledLen = (vid->nStride * vid->nSliceHeight * 3);
#endif
        TIME_END("fill buffer")

        TIME_START()
        if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(e->video_encode), buf) !=
             OMX_ErrorNone) {
            printf("Error emptying buffer!\n");
        }
        TIME_END("OMX_EmptyThisBuffer")
        TIME_START()
        out = ilclient_get_output_buffer(e->video_encode, 201, 1);
        TIME_END("ilclient_get_output_buffer")

        if (out != NULL) {
            if (out->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
               int i;
               for (i = 0; i < out->nFilledLen; i++)
                  printf("%x ", out->pBuffer[i]);
               printf("\n");
            }

            TIME_START()
            err = fwrite(out->pBuffer, 1, out->nFilledLen, e->outf);
            TIME_END("fwrite")
            if (err != out->nFilledLen) {
               printf("fwrite: Error emptying buffer: %d!\n", err);
            } else {
               //printf("Writing frame, len %u\n", out->nFilledLen);
            }
            if(e->fifoFile != NULL) {
                if(e->fifo < 0) {
                    e->fifo = open(e->fifoFile, O_WRONLY | O_NONBLOCK);
                    if(e->fifo >= 0) {
                        printf("Created H264 stream fifo at %s\r\n", e->fifoFile);
                    }
                }
                if(e->fifo >= 0) {
                    int writeRes = write(e->fifo, out->pBuffer, out->nFilledLen);
                    if(writeRes < 0 && errno != ENXIO) {
                        printf("Failed to write to video stream fifo, reason: %s\r\n", strerror(errno));
                    }
                }
            }
            out->nFilledLen = 0;
         } else {
            printf("Not getting it :(\n");
         }
         TIME_START()
         err = OMX_FillThisBuffer(ILC_GET_HANDLE(e->video_encode), out);
         TIME_END("OMG_FillThisBuffer")
         if (err != OMX_ErrorNone) {
            printf("Error sending buffer for filling: %x\n", err);
         }
    }
    frame ++;
}

int initEncode(RPIEncoder *e, const char *fileName, const char *fifoFile) {
    bcm_host_init();
    OMX_VIDEO_PARAM_PORTFORMATTYPE format;
    OMX_ERRORTYPE err;

    e->video_encode = NULL;

    memset(e->list, 0, sizeof(e->list));
    if((e->client = ilclient_init()) == NULL) {
        return -3;
    }
    if(OMX_Init() != OMX_ErrorNone) {
        ilclient_destroy(e->client);
        return -4;
    }
    err = ilclient_create_component(e->client, &e->video_encode, "video_encode", ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS | ILCLIENT_ENABLE_OUTPUT_BUFFERS);
    if(err != 0) {
        printf("ilclient_create_component() failed!\r\n");
        return -5;
    }
    e->list[0] = e->video_encode;

    // get current settings of video_encode component from port 200
    memset(&e->def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
    e->def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    e->def.nVersion.nVersion = OMX_VERSION;
    e->def.nPortIndex = 200;
    if (OMX_GetParameter
       (ILC_GET_HANDLE(e->video_encode), OMX_IndexParamPortDefinition,
        &e->def) != OMX_ErrorNone) {
        printf("%s:%d: OMX_GetParameter() for video_encode port 200 failed!\n",
             __FUNCTION__, __LINE__);
        return -6;
    }
    e->def.format.video.nFrameWidth = VIDEO_WIDTH;
    e->def.format.video.nFrameHeight = VIDEO_HEIGHT;
    e->def.format.video.xFramerate = VIDEO_FPS << 16;
    e->def.format.video.nSliceHeight = ALIGN_UP(e->def.format.video.nFrameHeight, 16);
    e->def.format.video.nStride = e->def.format.video.nFrameWidth;
    //e->def.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
    e->def.format.video.eColorFormat = OMX_COLOR_Format24bitBGR888;

    err = OMX_SetParameter(ILC_GET_HANDLE(e->video_encode), OMX_IndexParamPortDefinition, &e->def);
    if (err != OMX_ErrorNone) {
        printf
        ("%s:%d: OMX_SetParameter() for video_encode port 200 failed with %x!\n",
        __FUNCTION__, __LINE__, err);
        return -7;
    }

    memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 201;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;
    printf("OMX_SetParameter for video_encode:201...\n");
    err = OMX_SetParameter(ILC_GET_HANDLE(e->video_encode),
                        OMX_IndexParamVideoPortFormat, &format);
    if (err != OMX_ErrorNone) {
        printf
         ("%s:%d: OMX_SetParameter() for video_encode port 201 failed with %x!\n",
          __FUNCTION__, __LINE__, err);
        return -8;
    }

    OMX_VIDEO_PARAM_BITRATETYPE bitrateType;
    // set current bitrate to 1Mbit
    memset(&bitrateType, 0, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
    bitrateType.nSize = sizeof(OMX_VIDEO_PARAM_BITRATETYPE);
    bitrateType.nVersion.nVersion = OMX_VERSION;
    bitrateType.eControlRate = OMX_Video_ControlRateVariable;
    bitrateType.nTargetBitrate = 4000000;
    bitrateType.nPortIndex = 201;
    err = OMX_SetParameter(ILC_GET_HANDLE(e->video_encode),
                       OMX_IndexParamVideoBitrate, &bitrateType);
    if (err != OMX_ErrorNone) {
       printf
         ("%s:%d: OMX_SetParameter() for bitrate for video_encode port 201 failed with %x!\n",
          __FUNCTION__, __LINE__, err);
       return -9;
    }

    // get current bitrate
    memset(&bitrateType, 0, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
    bitrateType.nSize = sizeof(OMX_VIDEO_PARAM_BITRATETYPE);
    bitrateType.nVersion.nVersion = OMX_VERSION;
    bitrateType.nPortIndex = 201;

    if (OMX_GetParameter
        (ILC_GET_HANDLE(e->video_encode), OMX_IndexParamVideoBitrate,
        &bitrateType) != OMX_ErrorNone) {
       printf("%s:%d: OMX_GetParameter() for video_encode for bitrate port 201 failed!\n",
             __FUNCTION__, __LINE__);
       return -10;
    }
    printf("Current Bitrate=%u\n",bitrateType.nTargetBitrate);

    printf("encode to idle...\n");
    if (ilclient_change_component_state(e->video_encode, OMX_StateIdle) == -1) {
       printf
         ("%s:%d: ilclient_change_component_state(video_encode, OMX_StateIdle) failed",
          __FUNCTION__, __LINE__);
    }

    printf("enabling port buffers for 200...\n");
    if (ilclient_enable_port_buffers(e->video_encode, 200, NULL, NULL, NULL) != 0) {
       printf("enabling port buffers for 200 failed!\n");
       return -11;
    }

    printf("enabling port buffers for 201...\n");
    if (ilclient_enable_port_buffers(e->video_encode, 201, NULL, NULL, NULL) != 0) {
       printf("enabling port buffers for 201 failed!\n");
       return -12;
    }

    printf("encode to executing...\n");
    ilclient_change_component_state(e->video_encode, OMX_StateExecuting);

    e->outf = fopen(fileName, "w");
    e->fileName = fileName;
       if (e->outf == NULL) {
          printf("Failed to open '%s' for writing video\n", fileName);
          return -13;
     }

    if(fifoFile != NULL) {
        int fifoRes = mkfifo(fifoFile, 0666);
        if(fifoRes < 0) {
            e->fifo = -1;
            printf("Failed to create stream fifo at %s, reason: %s\r\n", fifoFile, strerror(errno));
        } else {
            e->fifoFile = fifoFile;
        }
        e->fifo = open(fifoFile, O_WRONLY | O_NONBLOCK);
    } else {
        e->fifo = -1;
    }

    return 1;
}

void closeEncode(RPIEncoder *e) {
    fclose(e->outf);
    if(e->fifo >= 0) {
        close(e->fifo);
    }
    ilclient_disable_port_buffers(e->video_encode, 200, NULL, NULL, NULL);
    ilclient_disable_port_buffers(e->video_encode, 201, NULL, NULL, NULL);
    ilclient_state_transition(e->list, OMX_StateIdle);
    ilclient_state_transition(e->list, OMX_StateLoaded);
    ilclient_cleanup_components(e->list);
    OMX_Deinit();
    ilclient_destroy(e->client);
}

