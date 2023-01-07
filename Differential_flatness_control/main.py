import importlib

# Reload the module to use the latest code
import unicycle_sim
importlib.reload(unicycle_sim)
from unicycle_sim import simulate_unicycle

# Run simulation
anim, fig = simulate_unicycle()
# Create animation
anim.save('Differential_Flatness_Control.gif')
