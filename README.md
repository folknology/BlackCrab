# BlackCrab
Next gen software for heterogeneous embedded development boards

Before the new boards are available we will Initially be targeting the existing MyStorm IceCore SOM for experimentation.

Required setup:
Rust 1.50 or a newer toolchain.
* itmdump >=0.3.1 (cargo install itm). Tested versions: 0.3.1.
* OpenOCD >=0.8. Tested versions: v0.9.0 and v0.10.0
* gdb-multiarch. Tested versions: 8.1
* cargo-binutils. Version 0.1.4 or newer.
* minicom on Linux and macOS.
* PuTTY on Windows.

### rustc & Cargo
Install rustup by following the instructions at https://rustup.rs.

If you already have rustup installed double check that you are on the stable channel and your stable toolchain is up to date. rustc -V should return a date newer than the one shown below:
```
 $ rustc -V
 rustc 1.50.0 (cb75ad5db 2021-02-10)
``` 

###  itmdump
 `cargo install itm`

Verify the version is >=0.3.1
```
 $ itmdump -V
 itmdump 0.3.1
```

### cargo-binutils

Install llvm-tools-preview

 `rustup component add llvm-tools-preview`

Install cargo-binutils

 `cargo install cargo-binutils`

Verify tools are installed
Run the following commands at your terminal
```
 cargo new test-size
 cd test-size
 cargo run
 cargo size -- -version`
```

you should see something like:

```
~
$ cargo new test-size
     Created binary (application) 'test-size' package

~
$ cd test-size

~/test-size (main)
$ cargo run
   Compiling test-size v0.1.0 (~/test-size)
    Finished dev [unoptimized + debuginfo] target(s) in 0.26s
     Running `target/debug/test-size`
Hello, world!

~/test-size (main)
$ cargo size -- -version
    Finished dev [unoptimized + debuginfo] target(s) in 0.00s
LLVM (http://llvm.org/):
  LLVM version 11.0.0-rust-1.50.0-stable
  Optimized build.
  Default target: x86_64-unknown-linux-gnu
  Host CPU: znver2
```
### Ubuntu 18.04 Linux (Including WSL) requirements:

```
 sudo apt-get install \
  gdb-multiarch \
  minicom \
  openocd
```
Add udev rules for ST-Link/open-OCD
```
 sudo vi /etc/udev/rules.d/99-openocd.rules
 ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE:="0666"
 sudo udevadm control --reload-rules
```
### Run OpenOCD

  `openocd -f interface/stlink-v2.cfg -f extra/IceCore.cfg`

_For WSL you will need to install a Windows executeable for OpenOCD as WSL cannot use the usb ST-Link tool._

you should see something like:

```
stlink-V2.cfg -f .\IceCore.cfg
Open On-Chip Debugger 0.10.0 (2019-07-15) [https://github.com/sysprogs/openocd]
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
WARNING: interface/stlink-v2.cfg is deprecated, please switch to interface/stlink.cfg
WARNING: interface/stlink-v2.cfg is deprecated, please switch to interface/stlink.cfg
Warn : Interface already configured, ignoring
Error: already specified hl_layout stlink
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
none separate

Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 2000 kHz
Info : STLINK V2J21S4 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.164644
Error: BUG: can't assert SRST
Info : STM32F730R8Tx.cpu: hardware has 8 breakpoints, 4 watchpoints
```

## To build and run BlackCrab
The first time you may need to add the correct target :

 `rustup target add thumbv7em-none-eabihf`

From then on you should just be able to build/run :

 `cargo run --features "stm32f730 rt usb_fs"`

  

