import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(0, 10, 500)
plt.plot(x, np.sin(x))
plt.xlabel("x")
plt.grid(True)
plt.show()