import numpy as np
import matplotlib.pyplot as plt

x = np.arange(-10, 0, 0.2)
print(x.size)
expx = np.exp(x)
y = 1 - np.log(1 + expx ** 2) / np.log(2)
plt.plot(x, y)
plt.show()