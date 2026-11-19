# Linux on the Samsung Galaxy A51 (SM-A515F)

Mainline Linux support for the Samsung Galaxy A51, an Exynos 9611 phone.
It boots into a full GPU-composited desktop: the kernel owns the display
controller, the MIPI link and the AMOLED panel, the Mali-G72 runs under
panfrost, and touchscreen, thermal sensors and the charger are all driven by
mainline drivers.

![Weston desktop with Xwayland, htop and a Wayland terminal](screenshots/a51-desktop.png)

*Weston with Xwayland: `htop` is an X11 client, the window next to it is a
native Wayland one, both composited on the GPU at 60 fps on the 1080x2400
panel.*

## Screenshots

| Sway | Phosh |
|---|---|
| <img src="screenshots/a51-sway-glmark.png" width="300"> | <img src="screenshots/a51-phosh.png" width="300"> |
| *Sway tiling `foot` next to `glmark2`, rendering a textured scene on the Mali-G72 through panfrost.* | *Phosh on the phone, with the blurred, translucent shell drawn by the GPU.* |


## Hardware

| | |
|---|---|
| SoC | Samsung Exynos 9611: 4x Cortex-A53 + 4x Cortex-A73, Mali-G72 MP3 |
| RAM | 6 GB |
| Display | 6.5" 1080x2400 AMOLED (Samsung EA8076), MIPI DSI, command mode |
| Storage | UFS (Android), microSD |
| Power | Samsung SM5713 charger, fuel gauge and MUIC, S2MPU09 PMIC |
| Input | Melfas mss100 touchscreen, GPIO keys |

## Status

Working:

- **Display** - DECON/IDMA/DPP scanout, DSIM host and D-PHY, panel bridge,
  brightness, DPMS, s2idle. 60 fps with vblank-driven page flips.
- **GPU** - panfrost on the Mali-G72, with a g3d power domain that gates the
  GPU through the vendor PMU sequences and TZPC SMC.
- **Touchscreen and keys** - mss100 multi-touch, power and volume keys.
- **Thermal** - Exynos 9610 TMU with eFuse trim calibration, three zones.
- **Charger** - input current limit, fast-charge current, float voltage and a
  charge-stop threshold with hysteresis.
- **USB** - DWC3 peripheral mode, used here as an ECM gadget (the root
  filesystem runs over it).
- **Storage** - microSD in non-UHS 50 MHz high-speed mode, UFS.
- **Desktops** - Weston with Xwayland, Sway, Phosh and KDE Plasma Mobile all
  render on the panel.

Not supported yet:

- **CPU frequency scaling** - the ACPM firmware does not service the DVFS
  channel, so the cores stay at their boot rate. This is the main reason heavy
  workloads feel slow.
- **CPU idle / deep sleep** - needs the PMUCAL core power down/up sequences;
  only s2idle works.
- **GPU DVFS** - no OPP table and no `mali-supply`, so the GPU clock is fixed.
- **USB host / OTG** - the controller is fixed to peripheral mode. The boost
  and MUIC switching are in place, the role switch is not wired up yet.
- Audio (ABOX), cameras, modem, WiFi/Bluetooth, NFC - untouched subsystems.

## Building

```sh
export CROSS_COMPILE=aarch64-linux-gnu-
make ARCH=arm64 exynos9611_a51_defconfig
make ARCH=arm64 -j$(nproc) Image dtbs modules
```

The device tree is `arch/arm64/boot/dts/exynos/exynos9611-a51.dts`; the shared
SoC description is `exynos9611.dtsi` next to it.

## Booting

The boot chain is the stock Samsung one, kept intact: the vendor bootloader
sets up the panel link, and the `uniLoader` payload then starts the kernel.
Flashing only touches the `boot` partition - the Android installation on the
other partitions stays untouched.

Short version:

```sh
# uniLoader is the small payload that starts the kernel from the stock
# bootloader; build it from my forked repository.
make ARCH=aarch64 CROSS_COMPILE=aarch64-linux-gnu-   # in uniLoader

magiskboot repack boot.img new-boot.img   # Image + dtb + uniLoader
dd if=new-boot.img of=/dev/block/by-name/boot bs=1M conv=fsync
```

## Notes on the port

Some parts of this SoC are fenced off by secure firmware, and the drivers
reflect what could be verified on hardware rather than what the vendor kernel
does:

- Power domains, DVFS and CPU idle go through the ACPM firmware interface on a
  shared mailbox; the mailbox and ACPM transports are ported, but the firmware
  only services the channels it already knows about.
- The display pipeline is driven by frame-done interrupts rather than the
  panel's tearing effect, which the panel never emits on this board.
- Shadow register updates are posted asynchronously. Waiting for the pipe to
  drain them inside the commit path delayed every flip event by one frame,
  which pinned vsynced compositors to half the panel refresh rate.

## Credits

Built on the work of the mainline Linux community, and on the vendor kernel
sources shipped by Samsung for the reference register layouts, PMU sequences
and firmware protocols.
