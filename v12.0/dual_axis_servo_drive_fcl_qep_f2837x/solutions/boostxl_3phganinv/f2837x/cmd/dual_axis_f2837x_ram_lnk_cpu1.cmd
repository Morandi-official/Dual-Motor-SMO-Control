//#############################################################################
//
// FILE:    dual_axis_f2837x_ram_lnk_cpu1.cmd.cmd
//
//#############################################################################
// $TI Release: MotorControl SDK v2.01.00.00 $
// $Release Date: Mon Nov 11 15:18:13 CST 2019 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within the memory map.
//
// The header linker files are found in <base>\2837x_headers\cmd
//
// For BIOS applications add:      f2837x_Headers_BIOS.cmd
// For nonBIOS applications add:   f2837x_Headers_nonBIOS.cmd
//
// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define
#ifdef CLA_C
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are.
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif //CLA_C


MEMORY
{
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN            : origin = 0x000000, length = 0x000002
   BOOT_RSVD        : origin = 0x000002, length = 0x0001AE     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x0001B0, length = 0x000250
   RAMM1            : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAMD0            : origin = 0x00C000, length = 0x000800
   RAMD1            : origin = 0x00C800, length = 0x000800
   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMGS0           : origin = 0x00D000, length = 0x001000
   RAMGS1           : origin = 0x00E000, length = 0x001000
   RAMGS2           : origin = 0x00F000, length = 0x001000
   RAMGS3           : origin = 0x010000, length = 0x001000
   RAMGS4           : origin = 0x011000, length = 0x001000
   RAMGS5           : origin = 0x012000, length = 0x001000
   RAMGS6           : origin = 0x013000, length = 0x001000
   RAMGS7           : origin = 0x014000, length = 0x001000
   RAMGS8           : origin = 0x015000, length = 0x001000
   RAMGS9           : origin = 0x016000, length = 0x001000
   RAMGS10          : origin = 0x017000, length = 0x001000
   RAMGS11          : origin = 0x018000, length = 0x001000
   RAMGS12          : origin = 0x019000, length = 0x001000
   RAMGS13          : origin = 0x01A000, length = 0x001000
   RAMGS14          : origin = 0x01B000, length = 0x001000
   RAMGS15          : origin = 0x01C000, length = 0x001000

   CPU1TOCPU2RAM    : origin = 0x03A000, length = 0x000800
   CPU2TOCPU1RAM    : origin = 0x03B000, length = 0x000800

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   CLA1_MSGRAMLOW   : origin = 0x001480,   length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500,   length = 0x000080
}


SECTIONS
{
   codestart        : > BEGIN
   // .text shares GS0-GS3 with .TI.ramfunc in the stock file, which leaves
   // no room for the RAM-resident FCL functions once the sensorless/eSMO +
   // field-weakening code grows .text. GS4, GS6-GS15 are otherwise unused
   // (only GS5 is taken, by SFRA), so spill .text into the upper blocks and
   // give .TI.ramfunc its own dedicated region (see below). No control code
   // changed - this is purely a memory-map fix.
   .text            : >> RAMGS0 | RAMGS1 | RAMGS2 | RAMGS3 | RAMGS7 |
                         RAMGS8 | RAMGS9 | RAMGS10 | RAMGS11 | RAMGS12 |
                         RAMGS13 | RAMGS14 | RAMGS15, ALIGN(4)
   .cinit           : > RAMLS1, ALIGN(4)
   .switch          : > RAMM0
   .reset           : > RESET, TYPE = DSECT /* not used, */
   .stack           : > RAMM1

#if defined(__TI_EABI__)
   .init_array      : >> RAMGS0 | RAMGS1 | RAMGS2 | RAMGS3, ALIGN(4)
   .bss             : > RAMLS6 | RAMLS7, ALIGN(4)
   .bss:output      : > RAMLS6 | RAMLS7
   .init_array      : > RAMM0
   .const           : > RAMLS6 | RAMLS7, ALIGN(4)
   .data            : > RAMLS6 | RAMLS7
   .sysmem          : > RAMLS6 | RAMLS7
#else
   .pinit           : >> RAMGS0 | RAMGS1 | RAMGS2 | RAMGS3, ALIGN(4)
   .ebss            : > RAMLS6 | RAMLS7, ALIGN(4)
   .econst          : > RAMLS6 | RAMLS7
   .esysmem         : > RAMLS6 | RAMLS7
#endif

   ramgs0 : > RAMGS0, type=NOINIT
   ramgs1 : > RAMGS1, type=NOINIT
   ESMO_COMPARE_LOG_DATA : > RAMGS6, ALIGN(4)
   SYNC_DOB_DATA : > RAMLS7, ALIGN(4)

   MSGRAM_CPU1_TO_CPU2 > CPU1TOCPU2RAM, type=NOINIT
   MSGRAM_CPU2_TO_CPU1 > CPU2TOCPU1RAM, type=NOINIT

    /* CLA specific sections */
   Cla1Prog         : >> RAMLS4 | RAMLS5, ALIGN(4)

   ClaData			: > RAMLS3, ALIGN(4)

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW, type=NOINIT
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH, type=NOINIT

   /* SFRA specific sections */
   SFRA_F32_Data	: > RAMGS5, ALIGN = 64

#ifdef CLA_C
   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS2

   .scratchpad      : > RAMLS2
   .bss_cla		    : > RAMLS2
   .const_cla	    : > RAMLS2
#endif //CLA_C

    // Dedicated RAM-function region. GS4 and GS6 are free (GS5 = SFRA), so
    // .TI.ramfunc (~0x5a8 today) gets its own 0x2000 home and no longer
    // competes with .text for GS0-GS3.
    .TI.ramfunc 	: >> RAMGS4 | RAMGS6, ALIGN(8)

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
