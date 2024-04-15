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

    def get_rssi_peak(self, frequency: float):
        frequency_span = 1e6
        all_frequencies, pxx_density = self.get_power_density_spectrum()

        indices_within_band = np.where(
            np.logical_and(all_frequencies >= (frequency - frequency_span),
                           all_frequencies <= (frequency + frequency_span)))[0]

        max_rssi_index = np.where(pxx_density == max(
            pxx_density[indices_within_band[0]:indices_within_band[-1]]))[0]

        # convert to dBm
        peak_rssi = 10 * np.log10(pxx_density[max_rssi_index][0])
        peak_frequency = all_frequencies[max_rssi_index][0]

        return peak_frequency, peak_rssi

    def get_power_density_spectrum(self):
        # - get iq samples and calculate power density
        # - sort frequencies vector and power_densities vector such that the
        # frequencies are increasing
        # - add center frequency so that frequencies vector contains the
        # absolute frequencies for the correpsonding power density

        samples = self.read_iq_samples()

        # FFT
        frequencies, pxx_density = scipy.signal.periodogram(
            samples,
            fs=self.sdr.sample_rate,
            nfft=1024,
        )

        frequencies = np.sort(frequencies)
        pxx_density = np.sort(pxx_density)

        frequencies = frequencies + self.sdr.center_freq

        return frequencies, pxx_density
