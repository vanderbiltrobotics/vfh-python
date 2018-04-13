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
        self.polar_histogram = [0] * num_bins


    def get_bin_index_from_angle(self, angle):
        """Returns index 0 <= i < nBins that corresponds to a. "Wraps" a around histogram."""
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360

        return angle // self.bin_width
#         return ((angle % self.num_bins) + self.num_bins) % nBins

    def get_middle_angle_of_bin(self, bin_index):
        """Returns the angle in the middle of the bin."""
        return (bin_index + 0.5) * self.bin_width


    def get_certainty_from_angle(self, angle):
        """Returns the value of the histogram for the specified bin."""
        return self.get_certainty(self.get_bin_index_from_angle(angle))

    def get_certainty(self, bin):
        """Returns the value of the histogram for the specified bin."""
        return self.polar_histogram[bin]


    def add_certainty_to_bin_at_angle(self, angle, delta_certainty):
        """Adds the passed value to the current value of the histogr1am grid."""
        self.polar_histogram[get_bin_index_from_angle(angle)] += delta_certainty


    def smooth_histogram(self, l):
        """Smoothing function that smooths the values of the histogram using a moving average."""
        for i in range(self.num_bins):
            culmsum = self.polar_histogram[i]*l

            k = 1
            for j in range(i-l, i):
                culmsum += k*self.polar_histogram[self.num_bins + (j % self.num_bins)]
                k += 1

            k = l
            for j in range(i+1, i+l+1):
                culmsum += k*self.polar_histogram[j % self.num_bins]
                k -= 1

            self.polar_histogram[i] = culmsum/(2*l + 1)


    def __str__(self):
        string = 'index, angle, certainty\n'
        for i, certainty in enumerate(self.polar_histogram):
#             print(i, i * self.bin_width, certainty)
            string += str(i) + ' ' + str(i * self.bin_width) + ' ' + str(certainty) + '\n'
#             print(str(i) + ',' + i * self.bin_width + ', ', certainty)
        return string


    def reset(self):
        self.histogram = [0] * self.num_bins
