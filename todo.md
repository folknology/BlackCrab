## BlackCrab Stream Pt.1 3/3/21 done
- [x] Initialise with cargo
- [x] Add .cargo/config
- [x] Add openocd.
- [x] Add openocd.gdb
- [x] Add memory.x
- [x] Add Cargo.toml
## BlackCrab Stream Pt.2 4/3/21 done
- [x] Get Mode & Status Leds operational (review)
- [x] Get Mode button working via interrupt
## BlackCrab Stream Pt.2 5/3/21 done
- [x] Get USB-CDC receiving and transmitting
- [x] Get USB-CDC receiving via interrupt calls
## BlackCrab Stream Pt.3-8 done
- [x] Port to RTIC (0.5.6)
- [x] FPGA Programming via SofSpi
- [x] FPGA Re-Programming via SofSpi
## BlackCrab Stream Pt.9-11 done
- [x] FPGA write leds via SPI
- [x] nMigen SPI Slave driving leds
- [x] Port to RTIC (0.6.0)
- [x] FPGA write leds via DSPI
- [x] nMigen DSPI Slave driving leds
- [x] nMigen PLL to wind up DSPI Clock
## BlackCrab Stream Ice LogicDeck
- [x] Assemble, test and bring-up ILD
- [x] Change Pins and Test programming STM32
- [x] Bring up USB and program Ice40
- [x] Test First Tile with Proto led trail
- [x] Create ILD PCF
- [x] Create ILD Amaranth Board Definition
- [x] Create Amaranth-HDL Lab Examples 
- [x] Move to PRobe-RS, Update RTIC and F7 HAl latest 
## Port BlackCrab to new ILB/BINxt
- [ ] Change Reset pin from PD10 to PB4
- [ ] Try new Ice SPI PRog pins from QSPI as bit bang QCK:PB2, QD0:PD11, QSS:PD6
- [ ] Port IceProgramming to QSPI peripheral
- [ ] Change Done pin input to PC13
- [ ] Change Mode pin PE5
- [ ] Mode Button is now boot/dfu button moved to pin PE3
- [ ] Added new  interrupt on PD3 
- [ ] Remove Status led, now redundant
- [ ] Remove WP and HLD pins as now redundant
- [ ] Add new SPI pins for STM Flash PB12-15:SS,SCK,SO,SI
- [ ] Add UART/USB-CDC support from STM32 <-> Ice40
- [ ] Add Flash programming support for Ice40 Images
- [ ] Move HDL examples from ArachePnr to NextPnr
- 