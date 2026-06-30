from datasets import load_dataset
import numpy as np

ds = load_dataset("zjandaman/record-test_20260617_153005", split="train")

gripper_action = np.array([x[-1] for x in ds["action"]])
gripper_state = np.array([x[-1] for x in ds["observation.state"]])

print("action min/max:", gripper_action.min(), gripper_action.max())
print("state min/max :", gripper_state.min(), gripper_state.max())

print("unique action values:", len(np.unique(np.round(gripper_action, 4))))
print("unique state values :", len(np.unique(np.round(gripper_state, 4))))