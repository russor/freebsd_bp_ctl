Notes from russor:

I've downloaded this driver from from https://www.silicom-usa.com/drivercat/bypass-1/
on May 31, 2022, Version 4.0.2 09/03/2021 (zip file on ftp dated Oct 19, 2021)

And changed it to update the timeout/untimeout calls to callout(9) as
timeout was removed from FreeBSD 13.0 and later. Interface scanning on each
ioctl was removed; it was panicing on my hosts, so I switched the load time
interface scanning to use ifunit_ref, added in FreeBSD 8, which I think may
better handle the reasoning behind if_scan? Additionally, ifdefs allowing
compilation on kernels earlier than FreeBSD 6 was removed.

As a side-note, if you've got a PEG4BPI or similar, older card that shows up
with vendor=0x1374, and not just the subvendor set; it might be possible to
adjust the eeprom to show up with Intel vendor and device ids, and then
it'll be much easier to use.  The simplest way I found to do that is not
super simple, but install the card in a system that can do IO-MMU based PCI
passthrough to a VM; run a Linux vm, with the NIC(s) passed through with the
expected Intel vendor and device id, and then you should be able to use
ethtool to adjust the EEPROM.  For my PEG4BPI, the desired vendor is 0x8086,
device 0x105E, and Initialization Control Word 1 (Word 0Ah) needed to be
adjusted, ethtool shows that word as offset 0x14 (low byte) and 0x15 (high
byte); bit zero of the low byte controls whether the device uses the default
vendor and device IDs or the values in the EEPROM; setting that to zero uses
the default values; the device then shows up as a PEG4BPI-SD and you no
longer need to modify this bpmod driver, or the em driver to detect it.

Original content from Silicom below:

    	Silicom FreeBSD Bypass Control Utility
		  
1. Compiling, installing and loading the software.

   Compiling the software:
	# make
		
   Installing the software in the system directory:
	# make install
	
   Loading the driver as a dynamic module:
	# kldload bpmod     
	
Note: 1. To load the driver automatically when the system is booted, please compile 
      and install the software, and, after that, edit /boot/loader.conf, and add   
      the following line:
	 bpmod_load="YES"
      2. Please load network driver before Bypass software loading. 		 
	      
2. Using the software.

Usage: bpctl_util <if_index|bus:slot.function> <command> [parameters]
       bpctl_util <info|help>
<if_index>   - interface name, for example, em0, or all for all Bypass-SD/TAP-SD Control devices
<command>    - bypass control command (see Commands List).
[parameters] - set_bypass_wd command:
                   WDT timeout interval, msec (0 for disabling WDT).
               set_bypass/set_bypass_pwoff/set_bypass_pwup/set_dis_bypass commands:
                   on/off for enable/disable Bypass
               set_std_nic command:
                   on/off for enable/disable Standard NIC mode
               set_tx command:
                   on/off for enable/disable transmit
               set_tpl command:
                   on/off for enable/disable TPL
               set_hw_reset command:
                   on/off for enable/disable hw_reset
               set_tap/set_tap_pwup/set_dis_tap commands:
                   on/off for enable/disable TAP
               set_disc/set_disc_pwup/set_dis_disc commands:
                   on/off for enable/disable Disc
               set_wd_exp_mode command:
                   bypass/tap/disc for bypass/tap/disc mode
               set_wd_autoreset command:
                   WDT autoreset interval, msec (0 for disabling WDT autoreset).
info         - print Program Information.
help         - print this message.
   Commands List:
is_bypass        - check if device is a Bypass/TAP controlling device
get_bypass_slave - get the second port participate in the Bypass/TAP pair
get_bypass_caps  - obtain Bypass/TAP capabilities information
get_wd_set_caps  - obtain watchdog timer settings capabilities
get_bypass_info  - get bypass/TAP info
set_bypass       - set Bypass mode state
get_bypass       - get Bypass mode state
get_bypass_change - get change of Bypass mode state from last status check
set_dis_bypass   - set Disable Bypass mode
get_dis_bypass   - get Disable Bypass mode state
set_bypass_pwoff - set Bypass mode at power-off state
get_bypass_pwoff - get Bypass mode at power-off state
set_bypass_pwup  - set Bypass mode at power-up state
get_bypass_pwup  - get Bypass mode at power-up state
set_std_nic      - set Standard NIC mode of operation
get_std_nic      - get Standard NIC mode settings
set_bypass_wd    - set watchdog state
get_bypass_wd    - get watchdog state
get_wd_time_expire - get watchdog expired time
get_wd_expire - get watchdog expired status
reset_bypass_wd - reset watchdog timer
set_tx      - set transmit enable / disable
get_tx      - get transmitter state (enabled / disabled)
set_tpl      - set TPL enable / disable
get_tpl      - get TPL state (enabled / disabled)
set_hw_reset          - set hw_reset enable / disable
get_hw_reset          - get hw_reset (enabled / disabled)
set_tap       - set TAP mode state
get_tap       - get TAP mode state
get_tap_change - get change of TAP mode state from last status check
set_dis_tap   - set Disable TAP mode
get_dis_tap   - get Disable TAP mode state
set_tap_pwup  - set TAP mode at power-up state
get_tap_pwup  - get TAP mode at power-up state
set_disc       - set Disc mode state
get_disc       - get Disc mode state
get_disc_change - get change of Disc mode state from last status check
set_dis_disc   - set Disable Disc mode
get_dis_disc   - get Disable Disc mode state
set_disc_pwup  - set Disc mode at power-up state
get_disc_pwup  - get Disc mode at power-up state
set_wd_exp_mode - set adapter state when WDT expired
get_wd_exp_mode - get adapter state when WDT expired
set_wd_autoreset - set WDT autoreset mode
get_wd_autoreset - get WDT autoreset mode
set_bp_manuf - set manufactory default (only for 71)

Example: bpctl_util em0 set_bypass_wd 5000
         bpctl_util all set_bypass on
         bpctl_util em0 set_wd_exp_mode tap
         bpctl_util 0b:00.0 get_bypass_info






   
