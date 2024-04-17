#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import argparse

from sdr.dvb_t import NooElec


def plot_power_spectrum_density(dvb_t_dongle):
    plt.ion()  # turn interactive mode on
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # init data
    frequencies, pxx_densities = dvb_t_dongle.get_power_density_spectrum()
    line, = ax.plot(frequencies, pxx_densities, 'b-')

    plt.axis([
        dvb_t_dongle.sdr.center_freq - 1.1e6,
        dvb_t_dongle.sdr.center_freq + 1.1e6, -140, 0
    ])
    plt.grid()
    plt.xlabel('Frequency [MHz]')
    plt.ylabel('Power [dB]')

    drawing = True
    line.set_xdata(frequencies)

    while drawing:
        try:
            # Busy-wait for keyboard interrupt (Ctrl+C)
            frequencies, pxx_densities = dvb_t_dongle.get_power_density_spectrum(
            )
            line.set_ydata(10 * np.log10(pxx_densities))

            fig.canvas.draw()
            plt.pause(0.01)

        except KeyboardInterrupt:
            print('Liveplot interrupted by user')
            drawing = False
    return True


def main(psd=True):
    dvb_t_dongle = NooElec()
    if psd:
        print(f'Live plotting of power density spectrum')
        plot_power_spectrum_density(dvb_t_dongle)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Live plotting utilities')
    parser.add_argument('--psd',
                        action='store_true',
                        help='Plot power density spectrum')  # on/off flag
    args = parser.parse_args()
    main(args.psd)
