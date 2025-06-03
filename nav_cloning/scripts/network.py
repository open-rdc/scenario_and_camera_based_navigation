from asyncore import write
from itertools import count
from platform import release
from pyexpat import features, model
import numpy as np
import matplotlib as plt
import os
import time
from os.path import expanduser


import torch
import torchvision
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, TensorDataset, Dataset, random_split
from torchvision import transforms
from torchvision.datasets import ImageFolder
import torch.optim as optim
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from collections import Counter
from yaml import load

# HYPER PARAM
BATCH_SIZE = 8
EPOCH = 5
PADDING_DATA = 7

class Perception(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.fc4 = nn.Linear(960, 512)
        self.fc5 = nn.Linear(512, 512)
        self.flatten = nn.Flatten()

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = self.flatten(x)
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        return x
    
class BranchedControl(nn.Module):
    def __init__(self, BRANCH=3):
        super().__init__()
        self.branches = nn.ModuleList([
            nn.Sequential(
                nn.Linear(512, 256),
                nn.ReLU(),
                nn.Linear(256, 1)
            ) for _ in range(BRANCH)
        ])

    def forward(self, joint, cmd):
        branch_outputs = [branch(joint) for branch in self.branches]
        branch_outputs = torch.stack(branch_outputs, dim=1).squeeze(-1)
        selected = (cmd * branch_outputs).sum(dim=1, keepdim=True)
        return selected

class FullBranchedModel(nn.Module):
    def __init__(self, BRANCH=3):
        super().__init__()
        self.perception = Perception()
        self.control = BranchedControl(BRANCH=BRANCH)

    def forward(self, image, cmd):
        p = self.perception(image)
        out = self.control(p, cmd)
        return out
    
class deep_learning:
    def __init__(self):
        # tensor device choiece
        self.device = torch.device(
            'cuda:0' if torch.cuda.is_available() else 'cpu')
        torch.manual_seed(0)
        self.net = FullBranchedModel().to(self.device)
        self.net.to(self.device)
        print(self.device)
        self.optimizer = optim.Adam(
            self.net.parameters(), eps=1e-2, weight_decay=5e-4)
        self.totensor = transforms.ToTensor()
        self.transform_color = transforms.ColorJitter(
            brightness=0.5, contrast=0.5, saturation=0.5)
        self.count = 0
        self.count_on = 0
        self.accuracy = 0
        self.loss_all =0.0
        self.max_freq = 0
        self.x_test = []
        self.c_test = []
        self.t_test = []
        self.criterion = nn.MSELoss()
        self.transform = transforms.Compose([transforms.ToTensor()])
        self.first_flag = True
        self.direction_counter = Counter()
        torch.backends.cudnn.benchmark = False

    def load_dataset(self, image_path, dir_path, vel_path):
        self.x_cat = torch.load(image_path)
        self.c_cat = torch.load(dir_path)
        self.t_cat = torch.load(vel_path)
        print("load_image:", self.x_cat.shape)
        print("load_dir:", self.c_cat.shape)
        print("load_vel:", self.t_cat.shape)
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        self.first_flag = False

    def call_dataset(self):
        if self.first_flag:
            return
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        return self.x_cat, self.c_cat, self.t_cat
        
    def make_dataset(self, img, dir_cmd, target_angle):        
        if self.first_flag:
            self.x_cat = torch.tensor(
                img, dtype=torch.float32, device=self.device).unsqueeze(0)
            self.x_cat = self.x_cat.permute(0, 3, 1, 2)
            self.c_cat = torch.tensor(
                dir_cmd, dtype=torch.float32, device=self.device).unsqueeze(0)
            self.t_cat = torch.tensor(
                [target_angle], dtype=torch.float32, device=self.device).unsqueeze(0)
            self.direction_counter[tuple(dir_cmd)] += 1
            self.first_flag = False

        # <To tensor img(x),cmd(c),angle(t)>
        x = torch.tensor(img, dtype=torch.float32,
                         device=self.device).unsqueeze(0)

        # <(Batch,H,W,Channel) -> (Batch ,Channel, H,W)>
        x = x.permute(0, 3, 1, 2)
        c = torch.tensor(dir_cmd, dtype=torch.float32,
                         device=self.device).unsqueeze(0)
        t = torch.tensor([target_angle], dtype=torch.float32,
                         device=self.device).unsqueeze(0)
        
        self.max_freq = max(self.max_freq, self.direction_counter[tuple(dir_cmd)])
        
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

        # <make dataset>
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        # <dataloader>
        train_dataset = DataLoader(dataset, batch_size=BATCH_SIZE, generator=torch.Generator(
            'cpu').manual_seed(0), shuffle=True)

        print("dataset_num:", len(dataset))
        return dataset, len(dataset), train_dataset

    def trains(self, iteration):
        if self.first_flag:
            return
        #self.device = torch.device('cuda')
        self.net.train()
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        train_dataset = DataLoader(dataset, batch_size=BATCH_SIZE, generator=torch.Generator(
            'cpu').manual_seed(0), shuffle=True)
        for i in range(iteration):
            for x_train, c_train, t_train in train_dataset:
                x_train.to(self.device, non_blocking=True)
                c_train.to(self.device, non_blocking=True)
                t_train.to(self.device, non_blocking=True)
                break
            
            self.optimizer.zero_grad()
            y_train = self.net(x_train, c_train)
            loss = self.criterion(y_train, t_train)
            loss.backward()
            self.loss_all = loss.item()
            self.optimizer.step()
        return self.loss_all

    def off_trains(self):
        # self.device = torch.device('cuda')
        print(self.device)
        # <Training mode>
        # <dataloder>
        dataset = TensorDataset(self.x_cat, self.c_cat, self.t_cat)
        train_dataset = DataLoader(dataset, batch_size=BATCH_SIZE,
        shuffle=True)

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
        action_value_training = self.net(x_act, c_act)

        return action_value_training.item(), self.loss_all

    def act(self, img, dir_cmd):
        self.net.eval()
        # <make img(x_test_ten),cmd(c_test)>
        x_test_ten = torch.tensor(
            img, dtype=torch.float32, device=self.device).unsqueeze(0)
        x_test_ten = x_test_ten.permute(0, 3, 1, 2)
        c_test = torch.tensor(dir_cmd, dtype=torch.float32,
                              device=self.device).unsqueeze(0)
        # <test phase>
        action_value_test = self.net(x_test_ten, c_test)

        for direction, count in self.direction_counter.items():
            print(f"Direction {direction}: {count}")

        return action_value_test.item()

    def save(self, save_path):
        # <model save>
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save({
            'model_state_dict': self.net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'loss': self.loss_all,
        }, path + '/model.pt')
        print("save_model")

    def save_tensor(self, input_tensor, save_path, file_name):
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save(input_tensor, path + file_name)
        print("save_dataset_tensor:",)

    def load(self, load_path):
        # <model load>
        checkpoint = torch.load(load_path)
        self.net.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.loss_all = checkpoint['loss']
        print("load_model =", load_path)

if __name__ == '__main__':
    dl = deep_learning()