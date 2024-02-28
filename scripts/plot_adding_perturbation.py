import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib

# # Data
# errors = np.array([0, 0.1, 0.2, 0.5, 1, 0, 0.1, 0.5, 1, 3, 5, 0, 0.2, 0.6, 1, 2, 0, 0.1, 0.2, 0.4, 0.6, 1])
# maneuverability_auc = np.array([0.95, 0.96, 0.95, 0.97, 0.98, 0.95, 0.95, 0.93, 0.87, 0.67, 0.64, 0.95, 0.95, 0.86, 0.75, 0.6, 0.96, 0.93, 0.85, 0.76, 0.64, 0.54])
# maneuverability_ap = np.array([0.99, 0.99, 0.99, 0.99, 1, 0.99, 0.98, 0.98, 0.95, 0.87, 0.86, 0.99, 0.99, 0.95, 0.9, 0.83, 0.99, 0.98, 0.94, 0.91, 0.86, 0.81])
# success_auc = np.array([0.95, 0.97, 0.97, 0.89, 0.8, 0.95, 0.97, 0.96, 0.9, 0.82, 0.84, 0.95, 0.94, 0.92, 0.92, 0.91, 0.95, 0.93, 0.93, 0.73, 0.76, 0.68])
# success_ap = np.array([0.97, 0.98, 0.98, 0.93, 0.85, 0.97, 0.98, 0.96, 0.91, 0.84, 0.84, 0.97, 0.96, 0.94, 0.92, 0.93, 0.96, 0.95, 0.95, 0.75, 0.81, 0.71])

# # Splitting the errors for different test conditions
# friction_errors = errors[:5]
# velocity_errors = errors[5:11]
# bias_errors = errors[11:16]
# all_together_errors = errors[16:]

# # Plotting
# fig, ax = plt.subplots(2, 2, figsize=(14, 10))

# # Friction Coefficient Noise
# ax[0, 0].plot(friction_errors, maneuverability_auc[:5], marker='o', label='Maneuverability AUC')
# ax[0, 0].plot(friction_errors, maneuverability_ap[:5], marker='o', label='Maneuverability AP')
# ax[0, 0].plot(friction_errors, success_auc[:5], marker='s', label='Success AUC')
# ax[0, 0].plot(friction_errors, success_ap[:5], marker='s', label='Success AP')
# ax[0, 0].set_title('Friction Coefficient Noise')
# ax[0, 0].set_xlabel('Noise max Magnitude')
# ax[0, 0].set_ylabel('metric value')
# # ax[0, 0].set_xlim(0, friction_errors[-1])
# # ax[0, 0].set_ylim(0.5, 1.05)
# ax[0, 0].legend()

# # Object and Gripper Velocity Noise
# ax[0, 1].plot(velocity_errors, maneuverability_auc[5:11], marker='o', label='Maneuverability AUC')
# ax[0, 1].plot(velocity_errors, maneuverability_ap[5:11], marker='o', label='Maneuverability AP')
# ax[0, 1].plot(velocity_errors, success_auc[5:11], marker='s', label='Success AUC')
# ax[0, 1].plot(velocity_errors, success_ap[5:11], marker='s', label='Success AP')
# ax[0, 1].set_title('Velocity Noise')
# ax[0, 1].set_xlabel('Noise max Magnitude (m/s)')
# ax[0, 0].set_ylabel('metric value')
# ax[0, 0].set_xlim(0, friction_errors[-1])
# ax[0, 1].legend()

# # Gripper Push Point Bias Noise
# ax[1, 0].plot(bias_errors, maneuverability_auc[11:16], marker='o', label='Maneuverability AUC')
# ax[1, 0].plot(bias_errors, maneuverability_ap[11:16], marker='o', label='Maneuverability AP')
# ax[1, 0].plot(bias_errors, success_auc[11:16], marker='s', label='Success AUC')
# ax[1, 0].plot(bias_errors, success_ap[11:16], marker='s', label='Success AP')
# ax[1, 0].set_title('Gripper Push Point Bias Noise')
# ax[1, 0].set_xlabel('Noise max Magnitude (m)')
# ax[1, 0].set_ylabel('metric value')
# ax[0, 0].set_xlim(0, friction_errors[-1])
# ax[1, 0].legend()

# # All Noises Together
# ax[1, 1].plot(all_together_errors, maneuverability_auc[16:], marker='o', label='Maneuverability AUC')
# ax[1, 1].plot(all_together_errors, maneuverability_ap[16:], marker='o', label='Maneuverability AP')
# ax[1, 1].plot(all_together_errors, success_auc[16:], marker='s', label='Success AUC')
# ax[1, 1].plot(all_together_errors, success_ap[16:], marker='s', label='Success AP')
# ax[1, 1].set_title('All Noises Together')
# ax[1, 1].set_xlabel('Noise max Magnitude')
# ax[0, 0].set_ylabel('metric value')
# ax[0, 0].set_xlim(0, friction_errors[-1])
# ax[1, 1].legend()

# # Show the plot
# plt.show()


########## Plot in a single figure
normalized_e_max = np.array([0, 0.1, 0.2, 0.5, 1, 0, 0.1, 0.5, 1, 0, 0.2, 0.6, 1, 0, 0.2, 0.4, 0.7, 1])
maneuverability_ap = np.array([0.99, 0.99, 0.99, 0.99, 1, 0.99, 0.98, 0.98, 0.95, 0.99, 0.99, 0.95, 0.9, 0.94, 0.902, 0.874, 0.864, .85])

# Splitting the errors for different test conditions
friction_errors = normalized_e_max[:5]
velocity_errors = normalized_e_max[5:9]
bias_errors = normalized_e_max[9:13]
force_error = normalized_e_max[13:]

# Plotting
fig, ax = plt.subplots(1, 1, figsize=(14, 10))

# Friction Coefficient Noise
ax.plot(friction_errors, maneuverability_ap[:5], marker='o', label='friction')
ax.plot(velocity_errors, maneuverability_ap[5:9], marker='o', label='velocity')
ax.plot(bias_errors, maneuverability_ap[9:13], marker='o', label='position')
ax.plot(force_error, maneuverability_ap[13:], marker='o', label='force')
# ax.set_title('Noise')
ax.set_xlabel('Normalized max Magnitude')
ax.set_ylabel('metric value')
# ax[0, 0].set_xlim(0, friction_errors[-1])
# ax[0, 0].set_ylim(0.5, 1.05)
# ax.legend()

# add grid
ax.grid()

# set y axis tick values
ax.set_yticks(np.arange(0.85, 1.01, 0.05))

# Show the plot
tikzplotlib.save("perturb-plot.tex")
plt.show()
