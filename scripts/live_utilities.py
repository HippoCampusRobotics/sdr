#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import argparse
import argcomplete

from sdr.dvb_t import NooElec


def live_plot_power_spectrum_density(dvb_t_dongle: NooElec):
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
    plt.xlabel('Frequency [Hz]')
    plt.ylabel('Power [dB? dBm?]')

    drawing = True
    line.set_xdata(frequencies)

    while drawing:
        try:
            frequencies, pxx_densities = dvb_t_dongle.get_power_density_spectrum(
            )
            line.set_ydata(10 * np.log10(pxx_densities))

            fig.canvas.draw()
            plt.pause(0.01)

        except KeyboardInterrupt:
            print('Live plot interrupted by user')
            drawing = False
    return True


def live_plot_tx_rss(dvb_t_dongle: NooElec, frequency: float):
    plt.ion()  # turn interactive mode on
    drawing = True
    counter = 0

    # save & plot last X samples
    num_history = 10
    rss_history = [-140.0] * num_history

    while drawing:
        try:
            counter += 1
            plt.clf()
            if counter > (num_history - 1):
                del rss_history[0]

            peak_frequency, peak_rss = dvb_t_dongle.get_rss_peak(frequency)
            rss_history.append(peak_rss)

            plt.plot(
                rss_history,
                'b.-',
                label=
                f'peak frequency = {peak_frequency / 1e6} MHz @ {peak_rss:.2f}',
            )
            plt.legend(loc='upper right')
            plt.ylim(-140, 0)
            plt.ylabel('RSS [dB? dBm?]')
            plt.grid()
            plt.pause(0.01)

        except KeyboardInterrupt:
            print('Live plot interrupted by user')
            drawing = False


def main(plot_type):
    dvb_t_dongle = NooElec()
    if plot_type == 'pds':
        print(f'Live plotting of power density spectrum')
        live_plot_power_spectrum_density(dvb_t_dongle)
    elif plot_type == 'tx_rss':
        frequency = 434.17e6
        print(f'Live plotting of transceiver rss for frequency {frequency}')
        live_plot_tx_rss(dvb_t_dongle, frequency)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Live plotting utilities')
    parser.add_argument('plot_type',
                        nargs='?',
                        choices=('pds', 'tx_rss'),
                        help='What type of live plot to plot')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    main(args.plot_type)
