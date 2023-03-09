The compensated temperature, compensated humidity, IAQ & Static IAQ are not supported in this example code 
because bsec expects the time to be ticking but during deepsleep the time resets to 0.

To overcome this the sleep duration can be added to the millis and passed to bsec from the example code.
Changes in bsec.c is also needed for this to work.