/* Quorum Window/Door Sensor
 * Quorum A-160 Window/Door Sensor Model HS-103
 * 
 * FCC ID: KHB-HS103-113
 * 
 * Transmits around 433.7 MHz
 * 
 * Bit width is ~500 us, OOK [1 = carrier on, 0 = no carrier]
 * Packet is 128 bits:
 *  - First 96 bits are 12 IOs (Address/Data) encoded with 8 bits each:
 *      Low       10001000 (Short Short)
 *      High      11101110 (Long  Long )
 *      Floating  10001110 (Short Long )
 *  - 32 bits of Sync: 10000000 00000000 00000000 00000000
 * 
 * First 5 IO are DIP switch settings 1 thru 6, inclusive:
 *   Floating means DIP bit set to 0
 *   Low      means DIP bit set to 1
 * DIP switch position 0 seems unused.
 * Remaining 7 IOs are fixed at 0110000
 * 
 * Packet is repeated a minimum of 4 times continuously (no gap).
 * 
 * Device only transmits when reed switch transitions from closed -> open.
 * 
 * Copyright (C) 2018 Lain <lain@hacktheinter.net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "rtl_433.h"
#include "data.h"
#include "util.h"

#define BIT_0 0x03 // Low
#define BIT_1 0x00 // High
#define BIT_F 0x02 // Float

#define IO_COUNT 12
#define NOMINAL_BIT_WIDTH_US 500

static int quorum_window_door_callback(bitbuffer_t *bitbuffer) {
    data_t *data;
    char time_str[LOCAL_TIME_BUFLEN];
    uint8_t *row;
    int r;
    int id = 0;
    
    int last_7_io = 0;
    const int EXPECTED_LAST_7_IO = 0x30; // 0110000

    //bitbuffer_print(bitbuffer);

    if (bitbuffer->num_rows < 4)
        return 0; // Truncated - device repeats 4x

    // 25 because 128 raw bits over the air, but PWM decode will see final 32 as 1 bit,
    // so that's actually 100 bits into the PWM decoder -> 4 bits in gives 1 bit out,
    // 100/4 = 25.
    r = bitbuffer_find_repeated_row(bitbuffer, 4, 25);

    if (r < 0)
        return 0;

    row = bitbuffer->bb[r];

    if (bitrow_get_bit(row, 24) != 1)
        return 0;   // Last bit is not '1' (Sync short pulse), invalid

    // First 5 IOs are DIP switch ID, next 7 IOs are fixed
    for (int i = 0; i < IO_COUNT; i++) {
        // PWM Decoder gives:
        //  Float 10
        //  Low   11
        //  High  00
        int bit = (bitrow_get_bit(row, i*2) << 1) | bitrow_get_bit(row, (i*2)+1);
        if (i < 5) {
            // ID bits (0-5)
            switch (bit) {
                case BIT_0: id |= 1<<i; break; // 0= DIP set to 1
                case BIT_F: id |= 0<<i; break; // F= DIP set to 0
                default: return 0; // Invalid state
            }
        } else {
            // Fixed bits (6-IO_COUNT)
            last_7_io <<= 1;
            switch (bit) {
                case BIT_0: last_7_io |= 0; break;
                case BIT_1: last_7_io |= 1; break;
                default: return 0; // Invalid state
            }
        }
    }

    // Validate that the last 7 IOs are 0110000, followed by sync bit ('1')
    if (last_7_io != EXPECTED_LAST_7_IO)
        return 0;

    local_time_str(0, time_str);
    data = data_make(
        "time",  "",           DATA_STRING, time_str,
        "model", "",           DATA_STRING, "Quorum A-160 Window/Door Sensor Model HS-103",
        "id",    "ID (5 bit)", DATA_INT,    id,
        NULL);
    data_acquired_handler(data);

    return 1;
}

static char *output_fields[] = {
    "time",
    "model",
    "id",
    NULL
};

/*
 * Analyzing pulses...
 * Total count:   25,  width: 12243             (49.0 ms)
 * Pulse width distribution:
 *  [ 0] count:   18,  width:   131 [129;133]   ( 524 us)
 *  [ 1] count:    7,  width:   383 [382;385]   (1532 us)
 * Gap width distribution:
 *  [ 0] count:   17,  width:   373 [372;376]   (1492 us)
 *  [ 1] count:    7,  width:   122 [121;123]   ( 488 us)
 * Pulse period distribution:
 *  [ 0] count:   24,  width:   504 [503;507]   (2016 us)
 * Level estimates [high, low]:  15892,    428
 * Frequency offsets [F1, F2]:    9107,      0  (+34.7 kHz, +0.0 kHz)
 * Guessing modulation: Pulse Width Modulation with fixed period
 * Attempting demodulation... short_limit: 257, long_limit: 377, reset_limit: 377, sync_width: 0
 * pulse_demod_pwm(): Analyzer Device
 * bitbuffer:: Number of rows: 1 
 * [00] {25} ae f0 ff 80 : 10101110 11110000 11111111 1
 */
r_device quorum_window_door = {
    .name          = "Quorum Window/Door Sensor",
    .modulation    = OOK_PULSE_PWM_PRECISE,
    .short_limit   =    NOMINAL_BIT_WIDTH_US, // Nominal width of '1' [us]
    .long_limit    =  3*NOMINAL_BIT_WIDTH_US, // Nominal width of '0' [us]
    .reset_limit   = 32*NOMINAL_BIT_WIDTH_US, // Maximum gap size before End Of Message [us].
    .gap_limit     =  5*NOMINAL_BIT_WIDTH_US, // Maximum gap size before new row of bits [us]
    .sync_width    = 0,    // Nominal width of sync pulse (optional) [us]
    .tolerance     = 0,    // Maximum deviation from nominal widths (optional, raw if 0) [us]
    .demod_arg     = 0,    // Do not remove any startbits
    .json_callback = &quorum_window_door_callback,
    .disabled      = 0,
    .fields        = output_fields
};
