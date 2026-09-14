import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 10, 500)
plt.plot(t, np.sin(t))
plt.xlabel("t")
plt.grid(True)
plt.show()