# Simple Game Console Based on AtMega87-20PU
This is a minimal gaming console based on two AtMega-MCUs.
The input is done via two old "Wii"-Nunchucks.
Output is given via monitor (VGA) and scores are displayed via 7-segment-displays.

The whole thing is rather old (2013), but I had a lot of fun building it.
In other words: Do not expect much regarding clean code etc.

I translate the original documentation into english using AI and put it below.
And yes.. I did a documentation.. but only because this was needed to publish the project on Heise.
The original documentation with all the nice pictures is also in the repo.

## Documentation (Translated by AI)

## Project Overview

This repository documents a simple video game console that runs the classic game **"Pong"** on a standard VGA monitor, controlled by two Wii Nunchuks. The hardware is powered by just two AtMega88-20PU microcontrollers, and the implementation showcases the technical challenges of VGA signal generation and game logic with limited resources.

## Features

**VGA Output**  
The console directly drives a VGA monitor in 640×480 @60Hz mode, outputting all graphics using the AtMega88 microcontroller. The graphics pipeline and Pong gameplay logic are written in Assembler for maximum performance.

**Controller Support**  
Two original Wii Nunchuks serve as input devices. Their joysticks and accelerometers allow paddle control, selectable via board jumpers. Communication is handled via I²C/TWI.

**Score Display**  
An array of four seven-segment displays shows the current score. Multiplexing and updates are managed in C, running on the second AtMega.

**Additional Controls**  
The board features jumpers for color/mode selection, a reset/start button, and three output pins as triggers for a simple sound system (optional, not included).

**Efficient Communication**  
The microcontrollers communicate via a custom UART protocol for fast score and control updates.

## Technical Highlights

### VGA Module

This unit is responsible for image generation, VGA signaling, game mechanics, and data transfer to the controller module. With the VGA protocol being extremely timing-sensitive, the implementation uses Assembler and overclocks the AtMega88 to 22.1145 MHz.  
The software architecture separates graphics generation (run at 5.5 MHz "pixel clock") and game logic into distinct processes, with a timer ISR acting as the scheduler.

### Controller Module

Implemented in C, this module polls the Nunchuks, manages scores, updates the seven-segment displays, and coordinates with the VGA module via UART. Nunchuk polling and game state updates are all performed within a single frame to maintain responsiveness.

### Hardware Design

Most circuit details are derived from the AtMega88 datasheet and well-known application notes (e.g., Philips AN97055 for level shifting). Special consideration is given to transistor pin layouts, with corrective diagrams provided for BC640 and BS108 variants.

## Notes on VGA Implementation

- **Pixel Clock Limitations:** The microcontroller cannot match the standard 25 MHz VGA pixel clock. Instead, output occurs at roughly 5.5 MHz, resulting in horizontally stretched pixels—but reliable display.
- **Memory Handling:** Graphics data is loaded into SRAM at startup for fast access; worklines and images are organized for efficient redrawing of game elements.
- **UART Datagram:** The UART protocol is custom-designed, using 8 data bits, one stop bit, and a baud rate of 32,143 Kbps.

## Images & Schematics

Key diagrams illustrating VGA timing, connector layouts, software architecture, datagram structure, and transistor pinouts are included in the documentation.

---

### Author

**Moritz Koal** (2013)

---

## Caution on Hardware Layout

When assembling, pay close attention to transistor pin assignments—especially for BC640 (swap collector and base) and BS108 (rotate drain and source by 180°). Refer to the included diagrams and datasheets for exact orientation.

---

## License

Feel free to modify, share, and build upon this project for educational or personal use.

