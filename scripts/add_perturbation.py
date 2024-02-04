import csv
import random

filename_friction = 'data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
new_filename_friction = 'data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_maneuver_labels_PlanePush_new.csv'
prname = 'PlanePush'
if prname == 'PlanePush':
    data = []
    with open(filename_friction, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for id, row in enumerate(csv_reader):
            data_id = []
            if id % 10 == 0:
                rand = random.uniform(-0.2, 1.0)
            for d in row:
                data_id.append(float(d))
            data_id.append(float(row[3])+rand)
            data.append(data_id)

# Save labels to a CSV file with headers
with open(new_filename_friction, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header + ['lateral_friction_coef_perturb_1.0',])
    writer.writerows(data)