/* JPEG Encoder Library
 * Copyright (c) 2006-2016 SIProp.org Noritsuna Imamura(noritsuna@siprop.org)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Sample Image Reader/Writer.
 * Spec:
 *       Data Attach Type: Address Base
 *       Image Format: 1-dimensional Sequential Array
 */
//#define DEBUG_ON_PC 1
//#ifdef DEBUG_ON_PC

#ifndef FLASH_EMU_H
#define FLASH_EMU_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*Added by Ali*/
#include "jpegencoder.h"

//Got from HM01B0_COMPRESS.h +++++++++++++++++++++++++++++++++++++++++
#define COMPRESSED_FILE_NAME "test_lowest4.jpg"
#define jpeg_imag_mem_size 100
#define NRF_FPU_USED 0
#define JPEG_16B 1

int open_files(char *file_original, char *file_converted);
int write_data(unsigned char *data, unsigned int len);
int read_data(unsigned int pos, unsigned char *data, unsigned int len);
int close_files(void);

#endif /*FLASH_EMU_H*/
