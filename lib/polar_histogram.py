"""
polar_histogram.py

A polar histogram means this, assuming bin_width=36
(therefore num_bins = 360 / 36 = 10):


index, corresponding_angle, histogram_angle
0, 0, 123
1, 36, 0
2, 72, 30
...
9, 324, 0

(equation: i * bin_width = angle)

However, we only keep index in a flat array for histograms, so we don't natively get/set by angle
but instead translate to and from angle.
"""
# PolarHistogram class creates an object to represent the Polar Histogram
class PolarHistogram:
    def __init__(self, num_bins):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            num_bins: Number of bins to divide polar space around robot into.
            bin_width: Angular width around robot included in each bin.
            histogram: Array storing the values of the polar histogram.
        """
        self.num_bins = num_bins
        self.bin_width = 360/num_bins
        self._polar_histogram = [0] * num_bins


    def wrap(self, bin_index):
        """Helper method for covering out of bounds bin_index."""
        while bin_index < 0:
            bin_index += self.num_bins

        while bin_index >= self.num_bins:
            bin_index -= self.num_bins

        return bin_index

    def get(self, bin_index):
        """custom getter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        return self._polar_histogram[bin_index]


    def set(self, bin_index, value):
        """custom setter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        self._polar_histogram[bin_index] = value

    def get_bin_index_from_angle(self, angle):
        """Returns index 0 <= i < nBins that corresponds to a. "Wraps" a around histogram."""
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360

        return int(angle // self.bin_width)

    def get_middle_angle_of_bin(self, bin_index):
        """Returns the angle in the middle of the bin."""
        bin_index = self.wrap(bin_index)
        return (bin_index + 0.5) * self.bin_width


    def get_certainty_from_angle(self, angle):
        """Returns the value of the histogram for the specified bin."""
        return self.get_certainty(self.get_bin_index_from_angle(angle))


    def add_certainty_to_bin_at_angle(self, angle, delta_certainty):
        """Adds the passed value to the current value of the histogr1am grid."""
        bin_index = self.get_bin_index_from_angle(angle)
        # print("polar_histogram: adding certainty %s to bin #%s = %s" % (delta_certainty, bin_index, angle))
        self._polar_histogram[bin_index] += delta_certainty


    def smooth_histogram(self, l):
        """Smoothing function that smooths the values of the histogram using a moving average."""
        smoothed_histogram = [0] * self.num_bins
        for k_i in range(self.num_bins):

            smoothed_histogram[k_i] = sum([(l - abs(k_i-l_i)) * self.get(l_i) for l_i in range(k_i-l+1, k_i+l)]) / (2*l+1)

        self._polar_histogram = smoothed_histogram

    def __str__(self):
        string = 'index, angle, certainty\n'
        for i, certainty in enumerate(self._polar_histogram):
            string += str(i) + ' ' + str(i * self.bin_width) + ' ' + str(certainty) + '\n'
        return string

    def get_angle_certainty(self):
        """Instead of (bin_index, certainty), return (angle, certainty) pairs."""
        return [(i * self.bin_width, certainty) for i, certainty in enumerate(self._polar_histogram)]


    def reset(self):
        self._polar_histogram = [0] * self.num_bins
