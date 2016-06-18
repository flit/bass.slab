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

#include "sequence_reader.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

SequenceReader::SequenceReader()
{
}

void SequenceReader::init()
{
    f_mount(&g_fs, "", 0);
}

SequenceInfo * SequenceReader::parse_file(FIL * fp)
{
    SequenceInfo * info = new SequenceInfo;

    f_lseek(fp, 0);

    while (!f_eof(fp))
    {
        static char s_buf[128];
        char * buf = f_gets(s_buf, sizeof(s_buf), fp);
        if (!buf)
        {
            break;
        }

        // Handle comment line.
        if (buf[0] == '#')
        {
            continue;
        }

        // Split into key and value.
        char * key = buf;
        char * value = buf;
        char * tmpbuf;
        bool done = false;
        while (!done)
        {
            char c = *buf++;
            switch (c)
            {
                case 0:
                case '\r':
                case '\n':
                    tmpbuf = buf - 1;
                    *tmpbuf = 0;
                    done = true;
                    break;
                case '=':
                    tmpbuf = buf - 1;
                    *tmpbuf = 0;
                    value = buf;
                    break;
                default:
                    break;
            }
        }
        if (value == key)
        {
            printf("Invalid syntax: %s\n", s_buf);
            continue;
        }

        // Scan for sequence nunber.
        if (strcmp(key, "tempo") == 0)
        {
            int tempo = atoi(value);
            info->tempo = tempo;
        }
        else
        {
            int sequenceNumber = atoi(key);
            if (sequenceNumber > 0 && sequenceNumber < 2)
            {
                int len = strlen(value) + 1;
                char * seq = new char[len];
                memcpy(seq, value, len);
                info->channels[sequenceNumber] = seq;
            }
        }

        printf("%s", buf);
    }

    return info;
}

uint32_t SequenceReader::scan_dir(const char * path, SequenceInfo ** head)
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
        printf("%s%s%s\n", path, filename, (info.fattrib & AM_DIR) ? "/" : "");

        SequenceInfo * lastSequence = 0;
        if (isdigit(filename[0]) && filename[1] == '.')
        {
            static FIL fp;
            FRESULT result;
            result = f_open(&fp, filename, FA_READ | FA_OPEN_EXISTING);
            if (result != FR_OK)
            {
                printf("Failed to open file\n");
                return 0;
            }

            printf("Processing sequence file: %s\n", filename);
            SequenceInfo * seqInfo = parse_file(&fp);
            f_close(&fp);

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
