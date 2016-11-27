/*
 * Copyright (c) 2016 Immo Software
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sequence_file_reader.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SequenceFileReader::SequenceFileReader()
{
}

void SequenceFileReader::init()
{
    f_mount(&g_fs, "", 0);
}

uint32_t SequenceFileReader::scan_dir(const char * path, SequenceInfo ** head)
{
    uint32_t sequenceCount = 0;
    FRESULT result;
    DIR dir;

    result = f_opendir(&dir, path);
    if (result != FR_OK)
    {
        return 0;
    }

    FILINFO info;
    char longFileNameBuffer[_MAX_LFN + 1];
    info.lfname = longFileNameBuffer;
    info.lfsize = sizeof(longFileNameBuffer);
    while (true)
    {
        result = f_readdir(&dir, &info);
        if (result != FR_OK || info.fname[0] == 0)
        {
            break;
        }
        const char * filename = info.lfname[0] ? info.lfname : info.fname;
        printf("%s%s%s (%d bytes)\n", path, filename, (info.fattrib & AM_DIR) ? "/" : "", info.fsize);

        SequenceInfo * lastSequence = 0;
        if (isdigit(filename[0]) && filename[1] == '.' && info.fsize > 0)
        {
            char * buf = (char *)malloc(info.fsize);
            if (!buf)
            {
                printf("Failed to allocate file buffer\n");
                continue;
            }

            static FIL fp;
            FRESULT result;
            result = f_open(&fp, filename, FA_READ | FA_OPEN_EXISTING);
            if (result != FR_OK)
            {
                printf("Failed to open file\n");
                return 0;
            }

            UINT btr = info.fsize;
            result = f_read(&fp, buf, btr, &btr);
            if (result != FR_OK)
            {
                printf("Failed to read file\n");
                return 0;
            }
            f_close(&fp);

            printf("Processing sequence file: %s\n", filename);
            SequenceInfo * seqInfo = parse(buf);
            if (!seqInfo)
            {
                continue;
            }

            if (!*head)
            {
                *head = seqInfo;
            }
            else
            {
                lastSequence->next = seqInfo;
            }
            lastSequence = seqInfo;
            ++sequenceCount;
        }
    }
    f_closedir(&dir);

    return sequenceCount;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
