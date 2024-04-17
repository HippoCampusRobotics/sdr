import numpy as np
import rtlsdr
import scipy.signal
from rclpy.impl import rcutils_logger


class NooElec(object):

    def __init__(self, gain=1, sample_rate=2.048e6, center_frequency=434.0e6):

        self.sdr = rtlsdr.RtlSdr()
        self.sdr.gain = gain
        self.sdr.sample_rate = sample_rate  # Hz or MS/s ?
        self.sdr.center_freq = center_frequency

        # TODO: woher kommt das??
        self.sample_size = 32
        self.read_size = 1024 * self.sample_size

        self.logger = rcutils_logger.RcutilsLogger(name="nooelec_logger")

    def read_iq_samples(self) -> np.ndarray:
        samples = self.sdr.read_samples(num_samples=self.read_size)
        return samples

    def get_rss_peak(self, frequency: float):
        frequency_span = 1e6
        all_frequencies, pxx_density = self.get_power_density_spectrum()

        indices_within_band = np.where(
            np.logical_and(all_frequencies >= (frequency - frequency_span),
                           all_frequencies <= (frequency + frequency_span)))[0]

        max_pxx_index = np.where(pxx_density == max(
            pxx_density[indices_within_band[0]:indices_within_band[-1]]))[0]
        max_pxx_index = max_pxx_index[0]

        # max_pxx = max(pxx_density)

        # self.logger.info(f'pxx_density: {pxx_density}')

        # max_pxx_index = np.where(pxx_density == max_pxx)
        # max_pxx_index = max_pxx_index[0]  # array to list
        # max_pxx_index = max_pxx_index[0]  # first list element

        # self.logger.info(f'max pxx: {max_pxx}')
        # self.logger.info(f'Max pxx index: {max_pxx_index}')

        # convert to dBm
        peak_rss = 10 * np.log10(pxx_density[max_pxx_index])
        peak_frequency = all_frequencies[max_pxx_index]

        return peak_frequency, peak_rss

    def get_power_density_spectrum(self):
        # - get iq samples and calculate power density
        # - sort frequencies vector and power_densities vector such that the
        # frequencies are increasing
        # - add center frequency so that frequencies vector contains the
        # absolute frequencies for the correpsonding power density

        samples = self.read_iq_samples()

        # FFT, pxx density in V**2/Hz
        frequencies, pxx_density = scipy.signal.periodogram(
            samples,  # time series of measurement values
            fs=self.sdr.sample_rate,  # sampling frequency
            nfft=1024,  # length of the fft
            return_onesided=False,  # two sided spectrum (we have complex data)
        )
        # we need to sort data so that frequencies increase
        # what indices would do this?
        sorted_indices = frequencies.argsort()
        # sort both arrays
        frequencies = frequencies[sorted_indices]
        pxx_density = pxx_density[sorted_indices]

        frequencies = frequencies + self.sdr.center_freq

        return frequencies, pxx_density
