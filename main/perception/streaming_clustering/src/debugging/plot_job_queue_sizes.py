import numpy as np
from matplotlib import pyplot as plt


def main():
    # input_file = "/tmp/file8ugI0Z"
    # input_file = "/tmp/fileKbuBtz"
    # input_file = "/tmp/fileThqdev"
    # input_file = "/tmp/filen5tPPy"
    # input_file = "/tmp/file8Y3xXY"
    # input_file = "/tmp/filepv8mIV"
    # input_file = "/tmp/filepPEOhr"
    # input_file = "/tmp/filerdCz3B"
    # input_file = "/tmp/fileTEeVL0"
    # input_file = "/tmp/filef9UdDJ"
    # input_file = "/tmp/file0TVZjb"
    # input_file = "/tmp/fileLU2i6f"
    # input_file = "/tmp/file8cIPKR"
    # input_file = "/tmp/fileJ8Roes"
    # input_file = "/tmp/fileybY8Uv"
    # input_file = "/tmp/filepzvlOe"
    # input_file = "/tmp/fileoU9h0u"
    # input_file = "/tmp/fileJhlmRB"
    # input_file = "/tmp/filesIPvPU"
    # input_file = "/tmp/filenQJzfv"
    input_file = "/tmp/fileF3DpY0"

    data = np.genfromtxt(input_file, delimiter=',')[:-1]  # last comma is also a element
    print(data.shape, data.dtype)
    data = data.reshape(-1, 5)
    plt.plot(data[:, 0], label='insertion')
    plt.plot(data[:, 1], label='segmentation')
    plt.plot(data[:, 2], label='association')
    plt.plot(data[:, 3], label='tree_combination')
    plt.plot(data[:, 4], label='publishing')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
