// Circuit Playground library to easily control a Circuit Playground
// using Firmata, ported over to Rust by Ethan Vazquez. Original code
// by Adafruit, authored by Tony DiCola. Original software licence
// for the Python version below:


// Circuit Playground PyMata helper class.
//
// This is not an example, rather it's a class to add Circuit Playground-specific
// commands to PyMata.  Make sure this file is in the same directory as the
// examples!
//
// Author: Tony DiCola
//
// The MIT License (MIT)
//
// Copyright 2016 Adafruit Industries
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:  The above copyright
// notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


use firmata::*;
use hex::*;
use serial-unix;
use serial-windows;
use std::env;

// Constants that define the Circuit Playground Firmata command values.
static CP_COMMAND: &str =               "0x40";     // Byte that identifies all Circuit Playground commands.
static CP_PIXEL_SET: &str =             "0x10";     // Set NeoPixel, expects the following bytes as data:
                                                    //  - Pixel ID (0-9)
                                                    //  - Pixel RGB color data as 4 7-bit bytes.  The upper
                                                    //    24 bits will be mapped to the R, G, B bytes.
static CP_PIXEL_SHOW: &str =            "0x11";     // Update NeoPixels with their current color values.
static CP_PIXEL_CLEAR: &str =           "0x12";     // Clear all NeoPixels to black/off.  Must call show pixels after this to see the change!
static CP_PIXEL_BRIGHTNESS: &str =      "0x13";     // Set the brightness of the NeoPixels, just like calling the
                                                    // NeoPixel library setBrightness function.  Takes one parameter
                                                    // which is a single byte with a value 0-100.
static CP_TONE: &str =                  "0x20";     // Play a tone on the speaker, expects the following bytes as data:
                                                    //  - Frequency (hz) as 2 7-bit bytes (up to 2^14 hz, or about 16khz)
                                                    //  - Duration (ms) as 2 7-bit bytes.
static CP_NO_TONE: &str =               "0x21";     // Stop playing anything on the speaker.
static CP_ACCEL_READ: &str =            "0x30";     // Return the current x, y, z accelerometer values.
static CP_ACCEL_TAP: &str =             "0x31";     // Return the current accelerometer tap state.
static CP_ACCEL_READ_REPLY: &str =      "0x36";     // Result of an accelerometer read.  Includes 3 floating point values (4 bytes each) with x, y, z
                                                    // acceleration in meters/second^2.
static CP_ACCEL_TAP_REPLY: &str =       "0x37";     // Result of the tap sensor read.  Includes a byte with the tap register value.
static CP_ACCEL_TAP_STREAM_ON: &str =   "0x38";     // Turn on continuous streaming of tap data.
static CP_ACCEL_TAP_STREAM_OFF: &str =  "0x39";     // Turn off streaming of tap data.
static CP_ACCEL_STREAM_ON: &str =       "0x3A";     // Turn on continuous streaming of accelerometer data.
static CP_ACCEL_STREAM_OFF: &str =      "0x3B";     // Turn off streaming of accelerometer data.
static CP_ACCEL_RANGE: &str =           "0x3C";     // Set the range of the accelerometer, takes one byte as a parameter.
                                                    // Use a value 0=+/-2G, 1=+/-4G, 2=+/-8G, 3=+/-16G
static CP_ACCEL_TAP_CONFIG: &str =      "0x3D";     // Set the sensitivity of the tap detection, takes 4 bytes of 7-bit firmata
                                                    // data as parameters which expand to 2 unsigned 8-bit bytes value to set:
                                                    //   - Type of click: 0 = no click detection, 1 = single click, 2 = single & double click (default)
                                                    //   - Click threshold: 0-255, the higher the value the less sensitive.  Depends on the accelerometer
                                                    //     range, good values are: +/-16G = 5-10, +/-8G = 10-20, +/-4G = 20-40, +/-2G = 40-80
                                                    //     80 is the default value (goes well with default of +/-2G)
static CP_CAP_READ: &str =              "0x40";     // Read a single capacitive input.  Expects a byte as a parameter with the
                                                    // cap touch input to read (0, 1, 2, 3, 6, 9, 10, 12).  Will respond with a
                                                    // CP_CAP_REPLY message.
static CP_CAP_ON: &str =                "0x41";     // Turn on continuous cap touch reads for the specified input (sent as a byte parameter).
static CP_CAP_OFF: &str =               "0x42";     // Turn off continuous cap touch reads for the specified input (sent as a byte parameter).
static CP_CAP_REPLY: &str =             "0x43";     // Capacitive input read response.  Includes a byte with the pin # of the cap input, then
                                                    // four bytes of data which represent an int32_t value read from the cap input.
static CP_SENSECOLOR: &str =            "0x50";     // Perform a color sense using the NeoPixel and light sensor.
static CP_SENSECOLOR_REPLY: &str =      "0x51";     // Result of a color sense, will return the red, green, blue color
                                                    // values that were read from the light sensor.  This will return
                                                    // 6 bytes of data:
                                                    //  - red color (unsigned 8 bit value, split across 2 7-bit bytes)
                                                    //  - green color (unsigned 8 bit value, split across 2 7-bit bytes)
                                                    //  - blue color (unsigned 8 bit value, split across 2 7-bit bytes)
static CP_IMPL_VERS: &str =             "0x60";     // Get the implementation version, 3 bytes of Major, Minor, Bugfix
static CP_IMPL_VERS_REPLY: &str =       "0x61";


// Accelerometer constants to be passed to set_accel_range.
static ACCEL_2G: u8  = 0;
static ACCEL_4G: u8  = 1;
static ACCEL_8G: u8  = 2;
static ACCEL_16G: u8 = 3;

// Constants for some of the board peripherals
static THERM_PIN: u8 =              0;              // Analog input connected to the thermistor.
static THERM_SERIES_OHMS: f64 =     10000.0;        // Resistor value in series with thermistor.
static THERM_NOMINAL_OHMS: f64 =    10000.0;        // Thermistor resistance at 25 degrees C.
static THERM_NOMIMAL_C: f64 =       25.0;           // Thermistor temperature at nominal resistance.
static THERM_BETA: f64 =            3950.0;         // Thermistor beta coefficient.
static CAP_THRESHOLD: u64 =         300;            // Threshold for considering a cap touch input pressed.
                                                    // If the cap touch value is above this value it is
                                                    // considered touched.

pub struct CircuitPlayground {
    win_board: Board<serial::windows::COMPort>,
    unix_board: Board<serial::unix::TTYPort>,
}

impl CircuitPlayground {
    pub fn new(port_id: &str) -> Result<CircuitPlayground, Box<dyn Error> {
        let mut sp = serial::open(port_id)?;

        sp.reconfigure(&|settings| {
            settings.set_baud_rate(Baud57600)?;
            settings.set_char_size(Bits8);
            settings.set_parity(ParityNone);
            settings.set_stop_bits(Stop1);
            settings.set_flow_control(FlowNone);
            Ok(())
        })?;

        let mut board = firmata::Board::new(Box::new(sp))?;

        println!("firmware version {}", board.firmware_version());
        println!("firmware name {}", board.firmware_name());
        println!("protocol version {}", board.protocol_version());

        let os_type = env::consts::OS;

        if os_type == "windows" {
            Ok(CircuitPlayground{win_board: board, unix_board: None})
        } else {
            Ok(CircuitPlayground{win_board: None, unix_board: board})
        }  
    } 
}