# SimpleButUsefulGPSDO
A simple but useful GPSDO project for using as a reference oscillator in projects (e.g. QO100, transverter, tansceiver TCXO) 

Initial design considerations :
- Arduino or MSP430 MCU will be used
- Cheap GPSs that provide 1PPS (e.g.Geo6m) or GPSs that provide frequency reference (e.g. Neo8m) will be used
- Should have more than 1 output (at least 2 or 3)
- The frequency reference should be adjustable (e.g. from 1MHz to 100MHz)
- Each output might have different frequency
- An LCD extension should be nice
- A raspberryPi extension with network Time Protocol server suppers might be brilliant

