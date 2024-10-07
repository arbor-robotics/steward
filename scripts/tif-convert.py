from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import cv2 as cv

"""
Converts a GeoTIFF file to a RAW file for Unity Terrain Toolbox.
"""

im = Image.open("schenley_fused.tif")
imarray = np.array(im, dtype=np.float64)

imarray[imarray > 350] = 350
imarray[imarray < 250] = 250

imarray[:, 1533] = imarray[:, 1532]
imarray[:, 1534] = imarray[:, 1533]

# normalize
imarray -= imarray.min()
imarray /= imarray.max()

# crop
imarray = imarray[:, -2354:]

# blur
imarray = cv.blur(imarray, (9, 9))

# convert to uint16
imarray *= 2**16
imarray = imarray.astype(np.uint16)

# resize to power of 2: 2^11 = 2048
imarray = cv.resize(imarray, (2048, 2048))

# imarray = imarray.astype(np.uint16)

# plt.hist(imarray.flatten())
print(imarray.shape)
print(imarray.dtype)
plt.imshow(imarray)
plt.show()
imarray.astype("uint16").tofile("image.raw")
