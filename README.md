# Raspberry Pi Detection Capture
A small, quick and informal project I made out of boredom to try radar setup on Linux, and to use as a fun device system. Captures pictures of intruders detected by a radar.

### How it works
Though perhaps not the most accurate way, my idea was to write a standard configuration to the radar, then read the buffer of the radar to determine a detection. If data is found, (non empty buffer), then take a photo via external camera and CV2, and store it somewhere. Trial and error would be needed to determine the optimal bytes length to determine if the radar has read an actual person or not to improve this system. v2 checks a complete TLV and bytebuffer process to actually parse the data. This can also potentially be used to determine via point cloud if a human is seen or random objects, and perhaps use TLV type to help us here. Feel free to contribute via PR!

### Notes
Though set up for Linux, I added a Windows config too. Just make sure to set UART ports based on your own machine.

