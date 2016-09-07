
rm -f  kernel.signed  kernel.itb

mkimage -f kernel.its kernel.itb

vbutil_kernel --pack kernel.signed \
      --keyblock /usr/share/vboot/devkeys/kernel.keyblock \
      --version 1 \
      --signprivate /usr/share/vboot/devkeys/kernel_data_key.vbprivk \
      --config kernel.flags  --vmlinuz kernel.itb --arch arm


#sudo dd if=kernel.signed of=/dev/sdc1
# mark mmcblk0p4 for a single try at high priority:
#sudo cgpt add -i 1 -S 1 -T 5 -P 12 /dev/sdc

# xserver-xorg-input-synaptics
# synclient TapButton1=1 TapButton2=3 TapButton3=0
