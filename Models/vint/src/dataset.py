import os
import pickle
import numpy as np
import torch
from torch.utils.data import Dataset
from PIL import Image
import torchvision.transforms as transforms

class ViNTDataset(Dataset):
    def __init__(self, root_dir, context_size=5, len_traj_pred=5, transform=None):
        self.root_dir = root_dir
        self.context_size = context_size
        self.len_traj_pred = len_traj_pred
        self.transform = transform or transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        self.samples = []

        traj_dirs = [d for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d))]
        for traj_dir in traj_dirs:
            traj_path = os.path.join(root_dir, traj_dir)
            with open(os.path.join(traj_path, 'traj_data.pkl'), 'rb') as f:
                traj_data = pickle.load(f)
                positions = traj_data['positions']
                yaws = traj_data['yaw']
                command = traj_data.get('command', 0)  # Assuming command exists

            num_images = len([f for f in os.listdir(traj_path) if f.startswith('img')])
            N = num_images
            if N != positions.shape[0]:
                raise ValueError(f"Mismatched images and positions in {traj_dir}")

            valid_t = range(self.context_size, N - self.len_traj_pred)
            for t in valid_t:
                self.samples.append({
                    'traj_path': traj_path,
                    't': t,
                    'command': command,
                    'positions': positions,
                    'yaws': yaws
                })

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]
        traj_path = sample['traj_path']
        t = sample['t']
        command = sample['command']
        positions = sample['positions']
        yaws = sample['yaws']
        N = positions.shape[0]

        # Load context images
        context_images = []
        for i in range(t - self.context_size, t + 1):
            img_name = f"img{i+1}.png"
            img_path = os.path.join(traj_path, img_name)
            image = Image.open(img_path).convert('RGB')
            if self.transform:
                image = self.transform(image)
            context_images.append(image)
        obs_img = torch.cat(context_images, dim=0)  # [3*(context_size+1), H, W]

        # Distance target: Euclidean distance to the goal (last position)
        current_pos = positions[t]
        goal_pos = positions[-1]
        distance_target = np.linalg.norm(current_pos - goal_pos)
        distance_target = torch.tensor(distance_target, dtype=torch.float32)

        # Action target: relative positions and yaws
        action_target = []
        for i in range(t + 1, t + self.len_traj_pred + 1):
            if i >= N:
                delta_pos = np.zeros(2)
                yaw = 0.0
            else:
                delta_pos = positions[i] - positions[t]
                yaw = yaws[i]
            action_target.append([delta_pos[0], delta_pos[1], yaw])
        action_target = torch.tensor(action_target, dtype=torch.float32)

        return (obs_img, torch.tensor(command)), (distance_target, action_target)