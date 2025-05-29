from typing_extensions import Self
import numpy as np
import matplotlib as plt
import os
import time
from os.path import expanduser


import torch
import torchvision
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, Dataset,Subset
from torchvision import transforms, models
from torchvision.datasets import ImageFolder
import torch.optim as optim
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
import random
import glob
#test
from torcheval.metrics.functional import multiclass_accuracy

# HYPER PARAM
BATCH_SIZE = 32
FRAME_SIZE = 16
EPOCH_NUM = 15

class Net(nn.Module):
    def __init__(self, n_channel, n_out):
        super().__init__()
    # <Network mobilenetv3>
        v3 = models.mobilenet_v3_large(weights='IMAGENET1K_V1')
        v3.classifier[-1]= nn.Linear(in_features=1280, out_features = 1280)
    # <CNN layer>
        self.v3_layer = v3
    #<LSTM + OUTPUT>
        self.lstm = nn.LSTM(input_size=1280,
                            hidden_size=512, num_layers=2, batch_first=True)
        self.output_layer = nn.Linear(512, n_out)

    def forward(self, x):
        # x's dimension: [B, T, C, H, W]
        # frameFeatures' dimension: [B, T, CNN's output dimension(1280)]
        frameFeatures = torch.empty(size=(x.size()[0], x.size()[1], 1280), device='cuda')
        for t in range(0, x.size()[1]):
            #<x[B,T,C,H,W]->CNN[B,T,1280]>
            #print("forword_x:",x.shape)
            frame = x[:, t, :, :, :]
            #print("forword_frame:",frame.shape)
            frame_feature = self.v3_layer(frame)
            #print(frame_feature.shape)
            #[B,seq_len,H]
            frameFeatures[:, t, :] = frame_feature
        #<CNN[B,T,1280] -> lstm[B,1280,512]>
        #print("lstm_in:",frameFeatures.shape)
        lstm_out, _ = self.lstm(frameFeatures)
        #<lstm[B,1280]-> FC[B,4,512]>
        # print("lstm_out:",lstm_out.shape)
        class_out = self.output_layer(lstm_out)
        # print("class_out",class_out.shape)
        class_out = torch.mean(class_out,dim=1)
        #print("class_out_mean",class_out.shape)
        return class_out
    
class deep_learning:
    def __init__(self):
        # <tensor device choice>
        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu')
        print(self.device)
        self.net = Net(n_channel=3, n_out=8)
        self.net.to(self.device)
        self.optimizer = optim.Adam(self.net.parameters(), eps=1e-2, weight_decay=5e-4)
        self.totensor = transforms.ToTensor()
        self.normalization = transforms.Compose([transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
        self.transform_color = transforms.ColorJitter(
            brightness=0.5, contrast=0.5, saturation=0.5)
        self.random_erasing = transforms.RandomErasing(
            p=0.25, scale=(0.02, 0.09), ratio=(0.3, 3.3), value= 'random')
        self.count = 0
        self.accuracy = 0
        self.results_train = {}
        self.results_train['loss'], self.results_train['accuracy'] = [], []
        self.acc_list = []
        self.datas = []       
        balance_weights = torch.tensor([1.0, 1.0, 10.0, 10.0, 1.0, 5.0, 7.5, 5.0]).to(self.device)
        self.criterion = nn.CrossEntropyLoss(weight=balance_weights)
        self.first_flag = True
        self.first_test_flag = True
        self.first_time_flag = True
        torch.backends.cudnn.benchmark = False
        torch.autograd.set_detect_anomaly(True)
        self.loss_all = 0.0
        self.intersection_test = torch.zeros(1,8).to(self.device)
        self.old_label = [0,0,0,0,0,0,0,0]
        self.diff_flag = False

    def make_dataset(self, img, intersection_label):
        # make tensor(T,C,H,W)
        if self.first_flag:
            self.x_cat = torch.tensor(
                img, dtype=torch.float32, device=self.device).unsqueeze(0)
            self.x_cat = self.x_cat.permute(0, 3, 1, 2)
            self.t_cat = torch.tensor(
                [intersection_label], dtype=torch.float32, device=self.device)
            if self.first_time_flag:
                self.x_cat_time = torch.zeros(1,FRAME_SIZE,3,48,64).to(self.device) 
                self.t_cat_time = torch.clone(self.t_cat)

            self.first_flag = False
            self.first_time_flag = False
        # <to tensor img(x),intersection_label(t)>
        x = torch.tensor(img, dtype=torch.float32, device=self.device).unsqueeze(0)
        # <(T,H,W,Channel) -> (T,Channel,H,W)>
        x = x.permute(0, 3, 1, 2)
        t = torch.tensor([intersection_label], dtype=torch.float32, device=self.device)
        print(intersection_label)
        if intersection_label == self.old_label:
            self.diff_flag = False
            self.x_cat = torch.cat([self.x_cat, x], dim=0)
            print("cat x_cat",self.x_cat.shape)
        else:
            self.first_flag = True
            self.diff_flag = True
            print("change label")
        self.old_label = intersection_label
       
        #<add tensor(B,C,H,W) to dateset_tensor(B,T,C,H,W)>
        if self.x_cat.size()[0] == FRAME_SIZE  and self.diff_flag ==False:
            print("make dataset")
            print("t_data:", t)
            #print("x_cat_time:",self.x_cat_time.shape,"x_cat_sq:",self.x_cat.unsqueeze(0).shape)
            self.x_cat_time = torch.cat((self.x_cat_time, self.x_cat.unsqueeze(0)), dim=0)
            self.t_cat_time = torch.cat((self.t_cat_time, t),dim=0)
            self.first_flag = True
    # <make dataset>
        print("train x =",self.x_cat_time.shape,x.device,"train t = " ,self.t_cat_time.shape,t.device)

        return self.x_cat_time,self.t_cat_time
        
    def training(self, load_x_tensor, load_t_tensor, load_flag):
        self.device = torch.device('cuda')
        print(self.device)

        if load_flag:
            load_x_tensor = torch.load(load_x_tensor)
            load_t_tensor = torch.load(load_t_tensor)

        print("x_tensor:", load_x_tensor.shape, "t_tensor:",load_t_tensor.shape)
        print("label info :", torch.sum(load_t_tensor, dim=0))
        dataset = TensorDataset(load_x_tensor, load_t_tensor)
        train_dataset = DataLoader(dataset, batch_size=BATCH_SIZE, generator=torch.Generator('cpu'), shuffle=True)

    # <training mode>
        self.net.train()
        self.train_accuracy = 0
    
    # <split dataset and to device>
        for epoch in range(EPOCH_NUM):
            print('epoch', epoch) 
            epoch_loss = 0.0
            epoch_accuracy = 0.0
            batch_loss = 0.0
            batch_accuracy = 0.0
            for x_train,t_label_train in train_dataset:
                x_train = x_train.to(self.device, non_blocking=True)
                t_label_train = t_label_train.to(self.device, non_blocking=True)
        # <use transform>
                # print("ddd=",len(x_train))
                # for i in range(len(x_train)):
                #     x_train[i, :, :, :, :] = self.transform_color(x_train[i,:, :, :, :])
                # x_train = self.transform_color(x_train)
                # x_train = self.random_erasing(x_train)
                #     # x_train =self.normalization(x_train)
        # <learning>
                self.optimizer.zero_grad()
                y_train = self.net(x_train)
                loss_all = self.criterion(y_train, t_label_train)
                # print("y = ",y_train.shape,"t=",t_label_train)
                self.train_accuracy += torch.sum(torch.max(y_train, 1)[1] == torch.max(t_label_train, 1)[1]).item()
                print("epoch:", epoch, "accuracy :", self.train_accuracy, "/", len(t_label_train),
                        (self.train_accuracy/len(t_label_train))*100, "%","loss :",loss_all.item())
                # self.writer.add_scalar("loss", loss_all.item(), self.count)
                # self.writer.add_scalar("accuracy",(self.train_accuracy/len(t_label_train))*100,self.count)
                loss_all.backward()
                self.optimizer.step()
                self.loss_all = loss_all.item()
                batch_loss += self.loss_all
                batch_accuracy += multiclass_accuracy(
                    input=torch.max(y_train,1)[1],
                    target=torch.max(t_label_train,1)[1],
                    num_classes=8,
                    average="micro"
                ).item()

                self.count += 1
                self.train_accuracy = 0
            epoch_loss = batch_loss / len(train_dataset)
            epoch_accuracy = batch_accuracy / len(train_dataset)
            print("epoch loss:", epoch_loss, "epoch accuracy:", epoch_accuracy)
            # self.writer.add_scalar("epoch loss", epoch_loss, epoch)
            # self.writer.add_scalar("epoch accuracy",epoch_accuracy,epoch)
        print("Finish learning")
        finish_flag = True

        return self.train_accuracy, self.loss_all

    def test(self, img):
        self.net.eval()
    # <to tensor img(x)>
        if self.first_test_flag:
            self.x_cat_test = torch.tensor(
                img, dtype=torch.float32, device=self.device).unsqueeze(0)
            self.x_cat_test = self.x_cat_test.permute(0, 3, 1, 2)
            self.first_test_flag = False
        
        x = torch.tensor(
            img, dtype=torch.float32, device=self.device).unsqueeze(0)
        x = x.permute(0, 3, 1, 2)
        self.x_cat_test = torch.cat([self.x_cat_test, x], dim=0)
        # print("x_test_cat:",self.x_cat_test.shape)
        if self.x_cat_test.size()[0] == FRAME_SIZE:
            self.intersection_test = self.net(self.x_cat_test.unsqueeze(0))
            print("s:",self.intersection_test.shape)
            self.x_cat_test = self.x_cat_test[1:]
        # print(x_test_ten.shape,x_test_ten.device,c_test.shape,c_test.device)
    # <test phase>        
        return torch.max(self.intersection_test, 1)[1].item()

    def save_tensor(self, dataset_tensor, save_path, file_name):
        os.makedirs(save_path)
        torch.save(dataset_tensor, save_path + file_name)
        print("save_path: ", save_path + file_name)

    def save(self, save_path):
        # <model save>
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save(self.net.state_dict(), path + '/model.pt')
    
    def load(self, load_path):
        # <model load>
        self.net.load_state_dict(torch.load(load_path))
        print(load_path)

if __name__ == '__main__':
    dl = deep_learning()
