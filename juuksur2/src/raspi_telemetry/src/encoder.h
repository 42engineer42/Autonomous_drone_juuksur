#pragma once 

#include "stdio.h"

#if defined(__x86_64__) || defined(__i386__)
typedef struct RPIEncoder {
} RPIEncoder;

#else

#include "bcm_host.h"
#include "ilclient.h"
typedef struct RPIEncoder {
    const char *fileName;
    const char *fifoFile;
    COMPONENT_T *list[5];
    ILCLIENT_T *client;
    COMPONENT_T *video_encode;
    OMX_PARAM_PORTDEFINITIONTYPE def;
    FILE *outf;
    int fifo;
} RPIEncoder;
#endif


#ifdef __cplusplus
extern "C" {
#endif

int pushFrame(RPIEncoder *e, int w, int h, int stride, char *data);
int initEncode(RPIEncoder *e, const char *fileName, const char *fifoName);
void closeEncode(RPIEncoder *e);

#ifdef __cplusplus
}
#endif
