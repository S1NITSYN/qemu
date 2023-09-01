# QEMU with PowerPC 476FP support

## Building

The simple steps to build QEMU for PowePC are:

```bash
    mkdir build
    cd build
    ../configure --target-list=ppc-softmmu \
        --disable-vnc --disable-sdl --disable-gnutls --disable-nettle --disable-gtk
    make
```

## Launching

MB115.01 board is available with PowerPC 476FP core. To start it you can use following command:

```bash
    sudo ./qemu-system-ppc \
        -M mb115.01 \
        -bios pc-bios/module_mb115_rumboot.bin \
        -drive file=pc-bios/module_mb115_u-boot.bin,if=mtd,format=raw \
        -monitor tcp::2345,server,nowait \
        -serial tcp::3555,server,nodelay,nowait \
        -gdb tcp::1234,server,nowait \
        -nic tap,model=greth,script=scripts/qemu-ifup,downscript=no
```

If you want to launch qemu with SD card use additional argument:

```bash
    -drive file=*path_to_sd_image_file*,if=sd,format=raw
```


Original QEMU readme is renamed to [README_original.rst](README_original.rst)
