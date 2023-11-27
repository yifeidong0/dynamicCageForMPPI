import torch
import math
import matplotlib.pyplot as plt

shape = (2,3,4)
rollout_cutdown_id = torch.tensor([2,3])
state = torch.arange(shape[0]*shape[1]*shape[2]).view(shape)
mask = torch.arange(shape[1]).expand(shape[0], -1) < rollout_cutdown_id.view(-1, 1)
result_tensor = state[mask].reshape(-1, shape[-1])
print('result_tensor',result_tensor)
torch.arange(result_tensor.shape[0])

# Create a list to store the summed predictions
summed_predictions = []
stability_cage = torch.arange(result_tensor.shape[0])
print('result_tensor',stability_cage)

# # Split stability_cage into groups based on state.shape[0]
# start_idx = 0
# for k in range(state.shape[0]):
#     group_size = rollout_cutdown_id[k]  # The size of the group for the current state[k]
#     end_idx = start_idx + group_size
#     group_predictions = stability_cage[start_idx:end_idx]  # Extract predictions for the group
#     summed_predictions.append(torch.sum(group_predictions))  # Sum the predictions
#     start_idx = end_idx  # Move the start index for the next group

# # Convert the list of summed predictions to a tensor
# cost_total = torch.stack(summed_predictions)
# print('cost_total',cost_total)

# Calculate cumulative sum of rollout_cutdown_id
cumulative_rollout = torch.cumsum(rollout_cutdown_id, dim=0)
print('cumulative_rollout', cumulative_rollout)

# Generate start indices of each group
start_indices = cumulative_rollout - rollout_cutdown_id
print('start_indices', start_indices)

# Create a range tensor for comparison
range_tensor = torch.arange(cumulative_rollout[-1])
print('range_tensor', range_tensor)

# Broadcast and create a mask for grouping
group_mask = (range_tensor >= start_indices[:, None]) & (range_tensor < cumulative_rollout[:, None])
print('group_mask', group_mask)

# Sum the elements of stability_cage for each group
cost_total = torch.sum(stability_cage[None, :] * group_mask, dim=1)
print('cost_total', cost_total)