To recompile the OSTC code you will need to install the following:
------------------------------------------------------------------

 - A Mercurial client, to download the source code and updates easily.
   TortoiseHg is free, and works well.

 - The Microchip MPLab IDE. This is free, and enable to recompile
   all the assembly code, link, and produce a .hex file.

 - If you want to modify the decompression algorithm, you will also
   need the MPLab C18 compiler. There is a demo free for the first
   30 days.
   If you don't modify p2_deco.c, you don't need the C18 compiler,
   see below.

 - The Tiny bootloader for windows, see the HW site.


Creating a working project:
---------------------------

 - Launch the MPLab IDE.

 - Create a new project (Project > Project Wizard...)
   - Choose device PIC18F4685
   - Select Microchip C18 Toolsuite   (if you installed the C18 compiler)
     or     Microchip MPASM Toolsuite (if not)
     --> Make sure the paths are corect.
   - Choose a name and a directoty.

   - Add the following files:
     <my_source_path>/code_part1/OSTC_code_asm_part1/18f4685_ostc_mkII.lkr
     <my_source_path>/code_part1/OSTC_code_asm_part1/MAIN.ASM
     <my_source_path>/code_part1/OSTC_code_c_part2/p2_deco.o

     (or use the .c instead of the .o if you have the C18 compiler)

   - If you want to compile C code, configure it:
     Menu: Project > Build Options... > Project > MPLab C18
         General: Default storage = Overlay (-sco)
         Optimization: Enable all

   - Make sure the IDE is configured to find the "clib.lib" file
     (needed even with precompiled p2_deco.o)
     Menu : Project > Build Options ... > Project > Directories
         Set «Search Directories» for : «Library Seach Path» to the path of the 
             “clib.lib” file (in c:\Program Files\Microship\MCC18\lib in my case)
         Set «Build Directory Policy» to «Assemble/Compile in source-file directory, link in output directory»

 - If you want to compile in DEBUG mode, select "Debug" in the build configuration
   menu in the top-bar of the IDE.
   It adds more safety tests into the code, so it is easier to spot bugs, 
   but generate firware NOT SUITABLE for diving !
   ==> Always revert to Release and recompile everything once debugging is done.

 - If you want a translated version (FRENCH, SPANISH, etc.), uncomment
   the corresponding #define in definitions.asm

 - Hit F10 to recompile everything.
   --> You should get a .hex file where you saved your project.


Installing the new firmware
---------------------------

 - Get the .hex file you want to flash onto the OSTC Mk2.

 - Connect the OSTC, wake it up.

 - Launch the Tiny bootloader. It should work for some time, trying to open
   the communication port.

 - Once it is ready, Click Browse to select the .hex file
   ==> DO NOT HIT Write Flash YET !

 - On the OSTC, go to the reset menu, select Reboot OSTC
   ==> DO NOT CONFIRM YET !

 - Click "Write Flash" button. Now you have 10sec to confirm on OSTC too.

 - The bootloader should say it found a 18F6485 device, and start uploading.
   The OSTC should have the blue and red led blinking rapidly.
   The upload time is ~ 20 secondes.

 - Once done, the OSTC finishes its reboot.

 - If you OSTC is stuck in some bad code, you can do the magic magnet reset
   instead of choosing the reboot menu, during the 10sec timeslot after
   starting bootloader write flash.



