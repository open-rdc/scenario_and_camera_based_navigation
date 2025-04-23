import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader, TensorDataset
from collections import Counter
import os
import time

# HYPERPARAMETERS
BATCH_SIZE = 64
BRANCH = 3
EPOCH = 20
PADDING_DATA = 3

class Perception(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=5, stride=2)
        self.conv2 = nn.Conv2d(32, 32, kernel_size=3, stride=1)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        self.conv4 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.conv5 = nn.Conv2d(64, 128, kernel_size=3, stride=2)
        self.conv6 = nn.Conv2d(128, 128, kernel_size=3, stride=1)
        self.conv7 = nn.Conv2d(128, 256, kernel_size=3, stride=1)
        self.conv8 = nn.Conv2d(256, 256, kernel_size=3, stride=1)
        self.flatten = nn.Flatten()
        self.fc = nn.Linear(8192, 512)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = F.relu(self.conv5(x))
        x = F.relu(self.conv6(x))
        x = F.relu(self.conv7(x))
        x = F.relu(self.conv8(x))
        x = self.flatten(x)
        x = F.relu(self.fc(x))
        return x

class BranchedControl(nn.Module):
    def __init__(self, num_branches=3):
        super().__init__()
        self.branches = nn.ModuleList([
            nn.Sequential(
                nn.Linear(512, 512),
                nn.ReLU(),
                nn.Linear(512, 256),
                nn.ReLU(),
                nn.Linear(256, 256),
                nn.ReLU(),
                nn.Linear(256, 1)
            ) for _ in range(num_branches)
        ])

    def forward(self, joint, cmd):
        branch_outputs = [branch(joint) for branch in self.branches]
        branch_outputs = torch.stack(branch_outputs, dim=1).squeeze(-1)
        selected = (cmd * branch_outputs).sum(dim=1, keepdim=True)
        return selected

class FullBranchedModel(nn.Module):
    def __init__(self, num_branches=3):
        super().__init__()
        self.perception = Perception()
        self.control = BranchedControl(num_branches=num_branches)

    def forward(self, image, cmd):
        p = self.perception(image)
        out = self.control(p, cmd)
        return out

class deep_learning:
    def __init__(self):
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        torch.manual_seed(0)
        self.net = FullBranchedModel().to(self.device)
        self.optimizer = optim.Adam(self.net.parameters(), lr=0.0002, betas=(0.7, 0.85))
        self.criterion = nn.MSELoss()
        self.first_flag = True
        self.direction_counter = Counter()
        self.loss_all = 0.0

    def make_dataset(self, img, dir_cmd, target_angle):
        x = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(self.device)
        c = torch.tensor(dir_cmd, dtype=torch.float32).unsqueeze(0).to(self.device)
        t = torch.tensor([target_angle], dtype=torch.float32).unsqueeze(0).to(self.device)

        if self.first_flag:
            self.x_cat, self.c_cat, self.t_cat = x, c, t
            self.direction_counter[tuple(dir_cmd)] += 1
            self.first_flag = False

        if dir_cmd == (0, 1, 0) or dir_cmd == (0, 0, 1):  
            for i in range(PADDING_DATA):
                self.x_cat = torch.cat([self.x_cat, x], dim=0)
                self.c_cat = torch.cat([self.c_cat, c], dim=0)
                self.t_cat = torch.cat([self.t_cat, t], dim=0)
            print("Padding Data")
            self.direction_counter[tuple(dir_cmd)] += 7
        else:
            self.x_cat = torch.cat([self.x_cat, x], dim=0)
            self.c_cat = torch.cat([self.c_cat, c], dim=0)
            self.t_cat = torch.cat([self.t_cat, t], dim=0)
            self.direction_counter[tuple(dir_cmd)] += 1

        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
        return dataset, len(dataset), loader

    def trains(self):
        if self.first_flag:
            return
        self.net.train()
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

        for x_train, c_train, t_train in loader:
            x_train = x_train.to(self.device)
            c_train = c_train.to(self.device)
            t_train = t_train.to(self.device)
            break

        self.optimizer.zero_grad()
        y_train = self.net(x_train, c_train)
        loss = self.criterion(y_train, t_train)
        loss.backward()
        self.loss_all = loss.item()
        self.optimizer.step()

        return self.loss_all

    def act_and_trains(self, img, dir_cmd, train_dataset):
        # <Training mode>
        self.net.train()

        # <split dataset and to device>
        for x_train, c_train, t_train in train_dataset:
            x_train.to(self.device, non_blocking=True)
            c_train.to(self.device, non_blocking=True)
            t_train.to(self.device, non_blocking=True)
            break

        # <use data augmentation>
        # x_train = self.transform_color(x_train)
            
        # <learning>
        self.optimizer.zero_grad()
        y_train = self.net(x_train, c_train)
            # y_train = y_train[torch.argmax(c_train)]
        # loss = self.loss_branch(c_train, t_train, y_train)
        loss = self.criterion(y_train, t_train)
        loss.backward()
        self.loss_all = loss.item() 
        self.optimizer.step()
        # <test>
        self.net.eval()
        x_act = torch.tensor(
            img, dtype=torch.float32, device=self.device).unsqueeze(0)
        x_act= x_act.permute(0, 3, 1, 2)
        c_act = torch.tensor(dir_cmd, dtype=torch.float32,
                              device=self.device).unsqueeze(0)
        with torch.no_grad():
            action_value_training = self.net(x_act, c_act)
        # action_value_training = action_value_training[torch.argmax(c_act)]

        return action_value_training.item(), self.loss_all

    def act(self, img, dir_cmd):
        self.net.eval()
        x = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(self.device)
        c = torch.tensor(dir_cmd, dtype=torch.float32).unsqueeze(0).to(self.device)
        with torch.no_grad():
            output = self.net(x, c)
        for direction, count in self.direction_counter.items():
            print(f"Direction {direction}: {count}")
        return output.item()

    def off_trains(self):
        # self.device = torch.device('cuda')
        print(self.device)
        # <Training mode>
        # <dataloder>
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        train_dataset = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

        for epoch in range(EPOCH):
            self.net.train()
            self.loss_all = 0.0
            count = 0
            for x_tensor, c_tensor, t_tensor in train_dataset:
                x_tensor = x_tensor.to(self.device, non_blocking=True)
                c_tensor = c_tensor.to(self.device, non_blocking=True)
                t_tensor = t_tensor.to(self.device, non_blocking=True)

            # <use data augmentation>
                # x_tensor = self.noise(x_tensor)
                # x_tensor = self.blur(x_tensor)
                # x_tensor = self.transform_color(x_tensor)
                # x_tensor = self.random_erasing(x_tensor)
            # <learning>
                self.optimizer.zero_grad()
                y_tensor = self.net(x_tensor, c_tensor)
                loss = self.criterion(y_tensor, t_tensor)
                loss.backward()
                self.optimizer.step()
                self.loss_all += loss.item()
                count += 1

            average_loss = self.loss_all / count
            print(f"Epoch {epoch+1}, Average Loss: {average_loss:.4f}")

    def call_dataset(self):
        if self.first_flag:
            return
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        return self.x_cat, self.c_cat, self.t_cat
    
    def save(self, save_path):
        path = save_path + time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(path)
        torch.save({
            'model_state_dict': self.net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'loss': self.loss_all,
        }, os.path.join(path, 'model.pt'))

    def save_tensor(self, input_tensor, save_path, file_name):
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save(input_tensor, path + file_name)
        print("save_dataset_tensor:",)

    def load(self, load_path):
        checkpoint = torch.load(load_path)
        self.net.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.loss_all = checkpoint['loss']

    def load_dataset(self, image_path, dir_path, vel_path):
        self.x_cat = torch.load(image_path)
        self.c_cat = torch.load(dir_path)
        self.t_cat = torch.load(vel_path)
        print("load_image:", self.x_cat.shape)
        print("load_dir:", self.c_cat.shape)
        print("load_vel:", self.t_cat.shape)
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        self.first_flag = False