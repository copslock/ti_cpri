

# VLAB Test to load pulsar0_cr5f-0 & c66x_0
# 
# Jonathan Bergsagel <jbergsagel@ti.com>
# Lokesh Vutla <lokeshvutla@ti.com>
# Robert Spark <r-spark@ti.com>
# 

import vlab
import os
import argparse

# directory with images to load
binary_directory = "../../../../binary/ex01_bios_2core_echo_test/bin/j721e_sim"

# If set, this must point to an absolute image path for M3 binary (eventually dmsc firmware)
m3_elf=""

parser = argparse.ArgumentParser(prog=__file__,
                                         prefix_chars="@",
                                         description='R5 & C66 IPC Loader - Keystone 3')
parser.add_argument('@@soc', help="SoC model j7es(default) or jacinto7", default="j7es")
parser.add_argument('@@ddr1', help="Size of RAM available on DDR1", default="0x00000000")

cmd_args=parser.parse_args(__args__)

keystone_args = ["--debugger-config=ccs"]

# DDR0 - 2GB: NOTE: we need to have PC memory corresponding to this
keystone_args += ["--ddr0_size=0x80000000"]
# DDR1 - The value can be specified in the commandline (default = 0 GB)
keystone_args += ["--ddr1_size=%s" % cmd_args.ddr1]

#keystone_args += ["--lock-step=mcu,soc1,soc2"]

variant = cmd_args.soc

mmc_config = []
if mmc_config != "" :
            keystone_args += ['--mmc='+','.join(mmc_config)]

#
# Change the DSP clocks (make them slower), in order to have relatively faster ARM sim time.
#
import keystone.c66_cluster.defines
import keystone.compute_cluster.defines
keystone.c66_cluster.defines.MODEL_PARAMS['c66p']['cycle_time_ps'] = 20000
keystone.compute_cluster.defines.MODEL_PARAMS['c7x']['cycle_time_ps'] = 20000

#
# Load the simulator
#
vlab.load("keystone.{variant}".format(variant=variant), args=keystone_args)
vlab.elaborate()

main0_r5_core0 = 'pulsar0_cr5f_0_proxy'
main0_r5_core0 = vlab.get_instance(main0_r5_core0)

c66x_0_core = 'platform.c66_cluster_0.c66p_0'
c66x_0_core = vlab.get_instance(c66x_0_core)

#
# Boot vecs, and Paths for binaries
# Workaround to use "before RESET" addresses (until we can load ATCM at address 0x0)...
reset_vecs_addr_1 = 0
reset_vecs_addr_2 = 0
# C66x boot addr top 22bits below converts to an address of: 0xa6200000
c66_reset_vecs_22bits = 0x00298800
#
mcu_bin_elf = os.path.join(binary_directory, "ex01_bios_2core_echo_test_mcu2_0_debug.xer5f")
c66x0_bin_elf = os.path.join(binary_directory, "ex01_bios_2core_echo_test_c66xdsp_1_debug.xe66")

# MCUSS R5_0 Config (MCU Island R5_0)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_CFG', 0x888)

# MAIN R5-0, Core0 Config (MAIN Domain Pulsar0, Core0)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_CFG', 0x888)

#
# Set the boot vector for each core, and ensure the HALT signal is set for each core (active-low)
#
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_BOOTVECT_LO', reset_vecs_addr_1)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_BOOTVECT_LO', reset_vecs_addr_2)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR2_CORE0_BOOTVECT_LO', c66_reset_vecs_22bits)
# Ensure the HALT signal is set for each R5 core
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_PMCTRL', 0)

vlab.run(100, blocking=True)

# Load C66x image before release of RESET
print("\n\nLoading C66x_0 image...\n\n")
vlab.load_image(c66x0_bin_elf, kind="elf", subject=c66x_0_core, load_data=True, load_symbols=True, set_pc=False)
#
# Put M3 into reset or out of reset if we have m3 firmware
#
if m3_elf == "" :
    print("\nDisabling M3..")
    vlab.write_port("platform.mcu_island.dmsc.cm3.RESETB", False)
else:
    print("\nLoading M3 firmware..." + m3_elf)
    vlab.write_port("platform.mcu_island.dmsc.cm3.RESETB", True)
    vlab.load_image(m3_elf, kind="elf", subject="mcu_island_dmsc_cm3_0_proxy", load_symbols=True)

#
# Crank up the dmtimer's frequency
#
vlab.write_port("platform.mcu_island.dmtimer0.CLKTIMER",  20000000000L)

print("\n\nResetting cores...\n")
vlab.write_port('platform.mcu_island.dmtimer0.RESET', True)
vlab.run(0, blocking=True)

# MCU1 (MCU Island R5, core0 & core1)
#vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[19]', 0x103)
#vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x2)
#vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[20]', 0x103)
#vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x2)

# MCU2 (MAIN Domain Pulsar0, core0 & core1)
vlab.write_register('platform.main_psc.PDCTL[24]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[93]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x1000000)
vlab.write_register('platform.main_psc.MDCTL[94]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x1000000)

# C66x_0
vlab.write_register('platform.main_psc.PDCTL[22]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[89]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3400000)
#
vlab.run(2000, blocking=True)
#
# Load binary images (only do this here after RESET is released, once loading ATCM at address 0x0 is fixed)
#
print("\n\nLoading R5 images...\n\n")
vlab.load_image(mcu_bin_elf, kind='elf', subject=main0_r5_core0, load_data=True, load_symbols=True, set_pc=False)

# Release the HALT signal on each R5 core to start running
# MCU1 (MCU Island R5s)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_PMCTRL', 1)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_PMCTRL', 1)
# MCU2 (MAIN Domain Pulsar0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_PMCTRL', 1)
#vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_PMCTRL', 1)

#
#
# Run the simulator
#
print "Starting with args:"
print keystone_args
vlab.run(0, blocking=True)
