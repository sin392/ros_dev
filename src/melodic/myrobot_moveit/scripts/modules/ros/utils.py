import numpy as np
from std_msgs.msg import MultiArrayDimension

# ref: https://qiita.com/kotarouetake/items/3c467e3c8aee0c51a50f
def numpy2multiarray(multiarray_type, np_array):
    """Convert numpy.ndarray to multiarray"""
    multiarray = multiarray_type()
    multiarray.layout.dim = [MultiArrayDimension(
        "dim%d" % i, np_array.shape[i], np_array.shape[i] * np_array.dtype.itemsize)
        for i in range(np_array.ndim)]
    multiarray.data = np_array.reshape(1, -1)[0].tolist()
    return multiarray


def multiarray2numpy(pytype, dtype, multiarray):
    """Convert multiarray to numpy.ndarray"""
    dims = [x.size for x in multiarray.layout.dim]
    res = np.array(multiarray.data, dtype=pytype).reshape(dims).astype(dtype)
    return res
