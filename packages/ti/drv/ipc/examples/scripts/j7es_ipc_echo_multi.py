
# VLAB Test to load A72_1_0, C66x_0, C66x_1, C7x, and all R5F cores for IPC echo comm. testing
# 
# Jonathan Bergsagel <jbergsagel@ti.com>
# Lokesh Vutla <lokeshvutla@ti.com>
# Robert Spark <r-spark@ti.com>
# 

import vlab
import struct
import os
import argparse

# directory with images to load
binary_directory = "../../../../binary/ipc_echo_test/bin/j721e_sim"
sciclient_dir = "../../../sciclient"

# If set, this must point to an absolute image path for M3 binary (eventually dmsc firmware)
m3_bin = os.path.join(sciclient_dir, "soc/V1/ti-sci-firmware-j721e-gp-vlab.bin")
r5_init_elf = os.path.join(sciclient_dir, "tools/ccsLoadDmsc/j721e/sciclient_ccs_init_mcu1_0_release.xer5f") 

parser = argparse.ArgumentParser(prog=__file__,
                                         prefix_chars="@",
                                         description='A72, C7x, C66x, & R5F IPC Loader - Keystone 3')
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

# Make sure DMSC and mcu_island_pulsar cr5f_0 and cr5f_1 are all in RESET
vlab.write_port("platform.mcu_island.pulsar.cr5f.RESETB[0]", False)
vlab.write_port("platform.mcu_island.pulsar.cr5f.RESETB[1]", False)
vlab.write_port("platform.mcu_island.dmsc.cm3.RESETB", False)
vlab.run(0, blocking=True)

# Ensure that the mcu island R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CFG', 0x9)

# Take mcu island R5 cores out of "HALT"
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_CFG', 0)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_CFG', 0)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_PMCTRL', 1)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_PMCTRL', 1)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_BOOTVECT_LO', 0)
vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_BOOTVECT_LO', 0)
vlab.run(0, blocking=True)

vlab.elaborate()

dmsc_core = 'mcu_island_dmsc_cm3_0_proxy'
dmsc_core = vlab.get_instance(dmsc_core)

mcu_r5_core0 = 'mcu_island_pulsar_cr5f_0_proxy'
mcu_r5_core0 = vlab.get_instance(mcu_r5_core0)

mcu_r5_core1 = 'mcu_island_pulsar_cr5f_1_proxy'
mcu_r5_core1 = vlab.get_instance(mcu_r5_core1)

main0_r5_core0 = 'pulsar0_cr5f_0_proxy'
main0_r5_core0 = vlab.get_instance(main0_r5_core0)

main0_r5_core1 = 'pulsar0_cr5f_1_proxy'
main0_r5_core1 = vlab.get_instance(main0_r5_core1)

main1_r5_core0 = 'pulsar1_cr5f_0_proxy'
main1_r5_core0 = vlab.get_instance(main1_r5_core0)

main1_r5_core1 = 'pulsar1_cr5f_1_proxy'
main1_r5_core1 = vlab.get_instance(main1_r5_core1)

c66x_0_core = 'platform.c66_cluster_0.c66p_0'
c66x_0_core = vlab.get_instance(c66x_0_core)

c66x_1_core = 'platform.c66_cluster_1.c66p_0'
c66x_1_core = vlab.get_instance(c66x_1_core)

c7x_core = 'platform.c7x_cluster_0.c7x'
c7x_core = vlab.get_instance(c7x_core)

# A72 cluster Name
cluster_id = 1
core_id = 0
cluster_name = 'ca72_' + str(cluster_id)
a72_core_name = cluster_name + '_{core_id}_proxy'
main_ca72_core0 = a72_core_name.format(core_id=core_id)
main_ca72_core0 = vlab.get_instance(main_ca72_core0)

#
# Boot vecs, and Paths for binaries
# Workaround to use "before RESET" addresses (until we can load ATCM at address 0x0)...
reset_vecs_addr_3 = 0
reset_vecs_addr_4 = 0
reset_vecs_addr_5 = 0
reset_vecs_addr_6 = 0
# C66x boot addr top 22bits below converts to an address of: 0xa6200000
c66_reset_vecs_22bits = 0x00298800
c66x2_reset_vecs_22bits = 0x0029C800
# C7x boot vector must be aligned to a 2MB offset, and it is shifted down by 2:
c7x_program_addr = 0xa8200000
c7x_reset_vecs_addr = c7x_program_addr >> 2
# A72 boot vector (also then shifted down by 2)
a72_program_addr = 0x80100000
a72_reset_vecs_addr = a72_program_addr >> 2
#
mcu1_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu1_0_debug.xer5f")
mcu2_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu1_1_debug.xer5f")
mcu3_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu2_0_debug.xer5f")
mcu4_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu2_1_debug.xer5f")
mcu5_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu3_0_debug.xer5f")
mcu6_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mcu3_1_debug.xer5f")
c66x0_bin_elf = os.path.join(binary_directory, "ipc_echo_test_c66xdsp_1_debug.xe66")
c66x1_bin_elf = os.path.join(binary_directory, "ipc_echo_test_c66xdsp_2_debug.xe66")
c7x_bin_elf = os.path.join(binary_directory, "ipc_echo_test_c7x_debug.xe71")
a72_bin_elf = os.path.join(binary_directory, "ipc_echo_test_mpu1_0_debug.xa72fg")

# MCUSS R5_0 Config (MCU Island R5_0)
# Ensure that MCU0 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CFG', 0x9)

# Leave MCU_R5_0 TCM set at its default value (BTCM)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_CFG', 0x888)

# MCUSS R5_1 Config (MCU Island R5_1)
# Ensure that MCU0 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CFG', 0x9)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_CFG', 0x888)

# MAIN R5-0, Core0 Config (MAIN Domain Pulsar0, Core0)
# Ensure that MCU1 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CFG', 0x9)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_CFG', 0x888)

# MAIN R5-0, Core1 Config (MAIN Domain Pulsar0, Core1)
# Ensure that MCU1 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CFG', 0x9)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_CFG', 0x888)

# MAIN R5-1, Core0 Config (MAIN Domain Pulsar1, Core0)
# Ensure that MCU2 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CFG', 0x9)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE0_CFG', 0x888)

# MAIN R5-1, Core1 Config (MAIN Domain Pulsar1, Core1)
# Ensure that MCU2 R5s are set to boot in 'lock-step' mode
#vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CFG', 0x9)
# Ensure both TCMs are enabled and that ATCM is at address 0x0
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE1_CFG', 0x888)

#
# Set the boot vector for each core, and ensure the HALT signal is set for each core (active-low)
#
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_BOOTVECT_LO', reset_vecs_addr_1)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_BOOTVECT_LO', reset_vecs_addr_2)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_BOOTVECT_LO', reset_vecs_addr_3)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_BOOTVECT_LO', reset_vecs_addr_4)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE0_BOOTVECT_LO', reset_vecs_addr_5)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE1_BOOTVECT_LO', reset_vecs_addr_6)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR2_CORE0_BOOTVECT_LO', c66_reset_vecs_22bits)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR3_CORE0_BOOTVECT_LO', c66x2_reset_vecs_22bits)
vlab.write_register('platform.mcu_island.dmsc.cc_boot_cfg.COREPAC0CORE0_BOOT_VECTOR_LO', a72_reset_vecs_addr)
vlab.write_register('platform.mcu_island.dmsc.cc_boot_cfg.COREPAC4CORE0_BOOT_VECTOR_LO', c7x_reset_vecs_addr)
# Ensure the HALT signal is set for each R5 core
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_PMCTRL', 0)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE0_PMCTRL', 0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE1_PMCTRL', 0)

#
# Put M3 into reset or out of reset if we have m3 firmware
#
vlab.display_terminal(vlab.terminal.mcu_island_usart1)
vlab.display_terminal(vlab.terminal.mcu_island_usart0)

if m3_bin == "" :
    print("\nDisabling M3..")
    vlab.write_port("platform.mcu_island.dmsc.cm3.RESETB", False)
else:
    print("\nLoading M3 firmware..." + m3_bin)
    vlab.write_port("platform.mcu_island.dmsc.cm3.BUS_CLOCK", 100000000)
    vlab.write_port("platform.mcu_island.dmsc.cm3.CLOCK", 200000000)
    vlab.load_image(m3_bin, offset = 0x40000, kind="bin", subject=dmsc_core, load_symbols=False)
    new_sp = vlab.read_memory(0x40000, 4, "mcu_island_dmsc_cm3_0_proxy", unpack="=L")
    new_pc = vlab.read_memory(0x40004, 4, "mcu_island_dmsc_cm3_0_proxy", unpack="=L")
    # Set the stack pointer (R13) and the PC to the correct values for SYSFW
    vlab.write_memory(0, bytearray(struct.pack('<I', new_sp)), "mcu_island_dmsc_cm3_0_proxy")
    vlab.write_memory(4, bytearray(struct.pack('<I', new_pc | 1)), "mcu_island_dmsc_cm3_0_proxy")
    vlab.write_port("platform.mcu_island.dmsc.cm3.RESETB", True)
    vlab.run(0, blocking=True)

#
# Crank up the dmtimer's frequency
#
vlab.write_port("platform.mcu_island.dmtimer0.CLKTIMER",  20000000000L)

print("\n\nResetting cores...\n")
vlab.write_port('platform.mcu_island.dmtimer0.RESET', True)

vlab.run(0, blocking=True)

print("\n\nLoading mcu0 R5 image...\n\n")
vlab.load_image(r5_init_elf, kind="elf", subject=mcu_r5_core0, load_data=True, load_symbols=True, set_pc=True)
vlab.write_port("platform.mcu_island.pulsar.cr5f.RESETB[0]", True)
#vlab.write_port("platform.mcu_island.pulsar.cr5f.RESETB[1]", True)
vlab.run(0, blocking=True)

# DMSC & MCU1 (MCU Island DMSC & R5, core0 & core1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[0]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[1]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[2]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[3]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[4]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[5]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x1)
vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[19]', 0x103)
vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x3)
#vlab.write_register('platform.mcu_island.wkup_psc.MDCTL[20]', 0x103)
#vlab.write_register('platform.mcu_island.wkup_psc.PTCMD', 0x3)

vlab.run(200, blocking=True)
vlab.write_register("mcu_island_pulsar_cr5f_0_proxy.PC", 0x41c04114)
#vlab.write_register("mcu_island_pulsar_cr5f_1_proxy.PC", 0x41c04114)

# Load C66x image before release of RESET
print("\n\nLoading C66x_0 image...\n\n")
vlab.load_image(c66x0_bin_elf, kind="elf", subject=c66x_0_core, load_data=True, load_symbols=True, set_pc=False)
print("\n\nLoading C66x_1 image...\n\n")
vlab.load_image(c66x1_bin_elf, kind="elf", subject=c66x_1_core, load_data=True, load_symbols=True, set_pc=False)
#
# Load C7x image before release of RESET
#print("\n\nLoading C7x DSP image...\n\n")
vlab.load_image(c7x_bin_elf, kind="elf", subject=c7x_core, load_data=True, load_symbols=False, set_pc=False)
#
# Load A72 image
print("\n\nLoading A72 image...\n\n")
vlab.load_image(a72_bin_elf, kind="elf", subject=main_ca72_core0, load_data=True, load_symbols=True, set_pc=True)

vlab.run(0, blocking=True)

# MCU2 (MAIN Domain Pulsar0, core0 & core1)
vlab.write_register('platform.main_psc.PDCTL[24]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[93]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x1000000)
vlab.write_register('platform.main_psc.MDCTL[94]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x1000000)

# MCU3 (MAIN Domain Pulsar1, core0 & core1)
vlab.write_register('platform.main_psc.PDCTL[25]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[96]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3000000)
vlab.write_register('platform.main_psc.MDCTL[97]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3000000)

# C66x_0
vlab.write_register('platform.main_psc.PDCTL[22]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[89]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3400000)
#

# C66x_1
vlab.write_register('platform.main_psc.PDCTL[23]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[91]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3C00000)
#

# C7x DSP
vlab.write_register('platform.main_psc.PDCTL[12]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[74]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3C01000)
#
# Finally, bring A72 Core 0 out of RESET...
print("\nResetting A72 core0...\n")
vlab.write_register('platform.main_psc.PDCTL[14]', 0x1)
vlab.write_register('platform.main_psc.PDCTL[15]', 0x1)
vlab.write_register('platform.main_psc.MDCTL[78]', 0x103)
vlab.write_register('platform.main_psc.MDCTL[80]', 0x103)
vlab.write_register('platform.main_psc.PTCMD', 0x3C0D000)
#
vlab.run(2000, blocking=True)
#

# Release MCU1 (MCU Island R5s) HALT signals
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE1_PMCTRL', 1)
#vlab.write_register('platform.mcu_island.dmsc.mcu_sec0.CLSTR0_CORE0_PMCTRL', 1)
#vlab.run(10, blocking=True)

#
# Load binary images (only do this here after RESET is released, once loading ATCM at address 0x0 is fixed)
#
print("\n\nLoading R5 images...\n\n")
#vlab.load_image(mcu1_bin_elf, kind="elf", subject=mcu_r5_core0, load_data=True, load_symbols=True, set_pc=True)
#vlab.load_image(mcu2_bin_elf, kind="elf", subject=mcu_r5_core1, load_data=True, load_symbols=True, set_pc=False)
vlab.load_image(mcu3_bin_elf, kind='elf', subject=main0_r5_core0, load_data=True, load_symbols=True, set_pc=False)
vlab.load_image(mcu4_bin_elf, kind='elf', subject=main0_r5_core1, load_data=True, load_symbols=True, set_pc=False)
vlab.load_image(mcu5_bin_elf, kind='elf', subject=main1_r5_core0, load_data=True, load_symbols=True, set_pc=False)
vlab.load_image(mcu6_bin_elf, kind='elf', subject=main1_r5_core1, load_data=True, load_symbols=True, set_pc=False)

# Release the HALT signal on each R5 core to start running

vlab.run(4000, blocking=True)

# MCU2 (MAIN Domain Pulsar0)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE0_PMCTRL', 1)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR0_CORE1_PMCTRL', 1)
# MCU3 (MAIN Domain Pulsar1)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE0_PMCTRL', 1)
vlab.write_register('platform.mcu_island.dmsc.main_sec0.CLSTR1_CORE1_PMCTRL', 1)
#

#
# Run the simulator
#
print "Starting with args:"
print keystone_args
vlab.run(0, blocking=True)
