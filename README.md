# dyClamp

## A real-time dynamic clamp (host) controller for the live [pyClamp](https://github.com/christianrickert/pyClamp) (client) interface

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2824830.svg)](https://doi.org/10.5281/zenodo.2824830)

**[dyClamp](https://github.com/christianrickert/dyClamp/)** is a further development of the [dynamic_clamp](https://github.com/nsdesai/dynamic_clamp) sketch with a focus on a robust serial communication between the Teensy and its controlling host computer: The current implementation allows scientists to alter the behavior of the [dynamic clamp](http://www.scholarpedia.org/article/Dynamic_clamp) system at runtime - by transmitting updates for calibration parameters, conductance values or by remotely triggering custom events. Furthermore, the low-latency design of [Teensyduino USB Serial Communication](https://www.pjrc.com/teensy/td_serial.html) enables a continuous flow of information from and to the host - with a minimal effect on the cycle times of the dynamic clamp system.

**[Video demonstration](https://github.com/christianrickert/pyClamp/blob/master/media/pyClamp.mkv?raw=true)** :movie_camera: (6.5 MiB, [H.264/MPEG-4 AVC](https://en.wikipedia.org/wiki/H.264/MPEG-4_AVC)) of **[dyClamp](https://github.com/christianrickert/dyClamp)** and **[pyClamp](https://github.com/christianrickert/pyClamp)** producing and monitoring a current command for the Axopatch 200B amplifier.

**_Cross-reference_:** **[pyClamp](https://github.com/christianrickert/pyClamp)** is a feature-complete graphical user interface written in Python to demonstrate the flexibility of this novel dynamic clamp implementation.

![dyClamp_protocol](https://user-images.githubusercontent.com/19319377/120081981-90570880-c07d-11eb-88b8-a7cc962c8e70.png)

## Serial communication

In short, the information exchanged between the Teensy and its host via the serial connection follows a simple formal convention: A new string of information begins with a carriage return character `<cr>` and ends with a linefeed character `<lf>`, while values embedded in this frame are separated by tabulator characters `<tab>`.

Example:
```
<cr> value1 <lf>
<cr> value1 <tab> value2 <lf>
<cr> value1 <tab> value2 <tab> value3 <lf>
```

Arduinos use [floating-point numbers](https://www.arduino.cc/reference/en/language/variables/data-types/float/) to represent decimal values internally - with up to seven digits of precision. However, the serial communication with UTF-8 encoding limits the precision of transferred values to two decimal digits. If you need to transmit values with a maximum precision, communicate those values as [integers](https://www.arduino.cc/reference/en/language/variables/data-types/int/) instead and divide them by the appropriate decimal power after the transfer.

### Serial command input

The Teensy will interpret information strings ("commands") from the controlling host based on `value1` and `value2`. The first value (index) indicates the type of command, while the second value represents an updated value or a pre-defined command case (command subtype).

- All positive command indices are indicating update commands for the array of calibration parameters.
```
/* This command changes the first value (at index 0) of the array of calibration parameters to "1.23". */
<cr> 1.0 <tab> 1.23 <lf>
```

- All negative command indices are indicating update commands for the array of conductance values.
```
/* This command changes the third value (at index 2) of the array of conductance values to "5.67". */
<cr> -3.0 <tab> 5.67 <lf>
```

- All command indices with a value of zero are generally interpreted as execution commands, with the second value indicating the subtype of the command.
```
/* This command does nothing by itself, except that it produces an echo to the host - see below.
   However, this command can still be useful to "ping" the Teensy when establishing a new connection. */
<cr> 0.0 <tab> 0.0 <lf>

/* Sends calibration parameters and conductance values to the host.
   These values can be interpreted by the host to confirm a successful transmission. */
<cr> 0.0 <tab> 1.0 <lf>
```

All command strings are echoed to the controlling host to indicate a successful transmission of the command.

### Serial command output

Besides the echo of a command string that is exclusively sent on-demand, the Teensy can additionally provide a continuous stream of information to its host upon request (live reports).
```
/* Toggles live reports of selected values on/off that are sent to the host continuously. */
<cr> 0.0 <tab> 2.0 <lf>
```
The Teensy will then start to report data values live representing the current state of internal values. The transmitted data can be useful to observe parameters during experiments or to debug new current models.
```
/* Continuous stream of information from the Teensy, reporting the latest values of
   the membrane potential [mV], the injected current [pA], and the cycle time [µs]. */
<cr> -65.1 <tab> 1.1 <tab> 9.0 <lf>
<cr> -64.9 <tab> 1.3 <tab> 10.0 <lf>
<cr> -64.7 <tab> 1.4 <tab> 9.0 <lf>
<cr> -65.4 <tab> 1.0 <tab> 9.0 <lf>
...
```

In order to reduce information overhead and therefore transmission latency, the information in the report strings is encoded positionally - in contrast to the command strings that use indices for the update of individual values.

Furthermore, the present implementation temporarily discontinues the generation of live reports when a new command string has arrived at its input buffer. This behavior avoids interpretation conflicts on the host side (i.e. a command string echo being interpreted as a live report with two values).

In addition to the serial communication tweaks mentioned above, **[dyClamp](https://github.com/christianrickert/dyClamp/)** is dynamically throttling its generation of live reports based on the availability of the serial output buffer. While this behavior does not guarantee a prompt transmission to the host system, it makes sure that data generation is synchronous with data representation.

## Deployment & Optimization

If you want to use **[dyClamp](https://github.com/christianrickert/dyClamp/)** in your dynamic clamp setup, you'll need recent versions of [Arduino](https://www.arduino.cc/en/Main/Software) to compile and upload the present sketch to your M4 Express.

- Arduino      
- Adafruit SAMD Core
- TinyUSB library
- elapsedMillis library for timekeeping

Using the **[OpenDynamicClamp 1.2.1 hardware](https://github.com/nsdesai/dynamic_clamp/tree/master/open-dynamic-clamp/odc-1.2)** you need to upload the dyClamp sketch from this repo, as it can handle the presence of a SAMD51 MCU during compilation. Also make sure to calibrate the output with the calibration sketch using the gain and offset pots. 

In the upload, you need to toggle 
 - "Overclocking" 200MHz
 - "Optimization" Faster -O3
 - "USB" TinyUSB

Especially the TinyUSB makes the system link much better with the pyClamp interface. In my testing, the cycle time bottoms out at 30µs. With the current code, no stronger optimization or ADC clock raise makes this run faster. Please feel free to go wild on the optimization of this code - maybe we can go down to 10µs?

**DONE:** The main tweaks so far are the overclocking of the chip, overclocking the ADC and implementing a hardware oversampling function. This enables faster processing and less noisy measurements. 

**TO-Do:** To cut cycle times more, a lookup table for the conductances could be used that is generated at the start of the experiment, thereby making it obsolete to calculate. The next step is to add more conductances and make them easier to implement by using only software inputs and omiting the software re-flash.

## Acknowledgements

I would like to thank [Cathy Proenza](https://medschool.cuanschutz.edu/physiology/faculty/cathy-proenza-phd), [Niraj Desai](https://github.com/nsdesai), and [Andrew Scallon](https://optogeneticsandneuralengineeringcore.gitlab.io/ONECoreSite/) for providing resources and valuable feedback during the development process.
Thanks from my side to [Christian Rickert](https://github.com/christianrickert/) for providing feedback and help during for porting the code from Teensy to the ODC 1.2.1 hardware!

## Development & Bug reports

If you would like to participate in the development, please [fork this repository](https://help.github.com/articles/fork-a-repo) to your GitHub account. In order to report a problem, please create a [new issue](https://help.github.com/articles/creating-an-issue/) in this repository.

Your feedback is welcome! Please contact me at [GitHub]((https://github.com/clckwrk22)) or via [e-mail](mailto:paul.j.wagner@fau.de).
