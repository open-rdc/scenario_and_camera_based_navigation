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
from torcheval.metrics.functional import multiclass_precision
from torcheval.metrics.functional import binary_accuracy
from torcheval.metrics import BinaryAccuracy
# HYPER PARAM
BATCH_SIZE = 32
FRAME_SIZE = 16
EPOCH_NUM = 20
CLIP_VALUE = 4.0

class Net(nn.Module):
    def __init__(self, n_channel, n_out):
        super().__init__()
    # <Network mobilenetv2>
        # v2 = models.mobilenet_v2(weights='IMAGENET1K_V1')
        v3 = models.mobilenet_v3_large(weights='IMAGENET1K_V1')
        v3.classifier[-1]= nn.Linear(in_features=1280, out_features = 1280)
        # v3.classifier[-1] = nn.Linear(in_features=1280, out_features=)
        # v2.classifier[1] = nn.Linear(
        #     in_features=1280, out_features=1280)
    #<LSTM + OUTPUT>
        self.lstm = nn.LSTM(input_size=1280,
                            hidden_size=512, num_layers=2, batch_first=True)
        self.output_layer = nn.Linear(512, n_out)
    # <CNN layer>
        self.v2_layer = v3

    def forward(self, x):
         # x's dimension: [B, T, C, H, W]

        # frameFeatures' dimension: [B, T, CNN's output dimension(1280)]
        frameFeatures = torch.empty(size=(x.size()[0], x.size()[1], 1280), device='cuda')
        for t in range(0, x.size()[1]):
            #<x[B,T,C,H,W]->CNN[B,T,1280]>
            #print("forword_x:",x.shape)
            frame = x[:,t, :, :,:]
            #print("forword_frame:",frame.shape)
            frame_feature = self.v2_layer(frame)
            #print(frame_feature.shape)
            #[B,seq_len,H]
            frameFeatures[:,t, :] = frame_feature
        #<CNN[B,T,1280] -> lstm[B,1280,512]>
        #print("lstm_in:",frameFeatures.shape)
        lstm_out,_ = self.lstm(frameFeatures)
        #<lstm[B,1280]-> FC[B,4,512]>
        # class_out = self.output_layer(lstm_out[:,-1,:])
        # print("lstm_out:",lstm_out.shape)
        # for f in range(0,lstm_out.size()[0]):
        class_out = self.output_layer(lstm_out)
        # print("class_out",class_out.shape)
        class_out =torch.mean(class_out,dim=1)
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
        self.transform_train = transforms.Compose([transforms.RandomRotation(10),
                                                   transforms.ColorJitter(brightness=0.3, saturation=0.3)])
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
        self.buffer_list = torch.zeros(1, 8).to(self.device)
        self.buffer_size = 7
        self.intersection_labels = []
        # # balance_weights = torch.tensor([1.0, 1.0,5.0,5.0,1.0,5.0,10.0,5.0]).to(self.device)
        # self.criterion = nn.CrossEntropyLoss(weight=balance_weights)
        #maech area
        balance_weights = torch.tensor([1.0, 1.0, 10.0, 10.0, 1.0, 5.0, 7.5, 5.0]).to(self.device)
        #mech+add area
        # balance_weights = torch.tensor([1.0, 39.5, 12.8, 13.8, 0, 5.1, 8.5, 5.9]).to(self.device)
        # 3camere
        # balance_weights = torch.tensor([1.0, 39.5, 14.0, 14.2, 0, 6.6, 7.0, 6.4]).to(self.device)
        self.criterion = nn.CrossEntropyLoss(weight=balance_weights)
        self.first_flag = True
        self.first_test_flag = True
        self.first_time_flag = True
        torch.backends.cudnn.benchmark = False
        # self.writer = SummaryWriter(log_dir='/home/takumi/catkin_ws/src/intersection_detector/runs')
        # torch.manual_seed(0)
        torch.autograd.set_detect_anomaly(True)
        self.loss_all = 0.0
        self.intersection_test = torch.zeros(1,8).to(self.device)
        self.old_label = [0,0,0,0,0,0,0,0]
        self.diff_flag = False

    def make_dataset(self, img, intersection_label):
        # self.device = torch.device('cpu')
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
        x = torch.tensor(img, dtype=torch.float32,
                         device=self.device).unsqueeze(0)
        # <(T,H,W,Channel) -> (T ,Channel, H,W)>
        x = x.permute(0, 3, 1, 2)
        # <(t dim [4]) -> [1,4] >
        t = torch.tensor([intersection_label], dtype=torch.float32,
                         device=self.device)
        print(self.old_label)
        #<>
        if intersection_label == self.old_label:
            self.diff_flag = False
            self.x_cat = torch.cat([self.x_cat, x], dim=0)
            print("cat x_cat!!",self.x_cat.shape)
        else:
            self.first_flag = True
            self.diff_flag = True
            print("change label")
        # <self.x_cat (B,C,H,W) = (8,3,48,64))>
        self.old_label = intersection_label
       
        #<add tensor(B,C,H,W) to dateset_tensor(B,T,C,H,W)>
        if self.x_cat.size()[0] == FRAME_SIZE  and self.diff_flag ==False:
            # <self.x_cat_time (B,T,C,H,W) = (8,8,3,48,64))>
            print("make dataset")
            print("t_data:",t)
            #print("x_cat_time:",self.x_cat_time.shape,"x_cat_sq:",self.x_cat.unsqueeze(0).shape)
            self.x_cat_time = torch.cat((self.x_cat_time, self.x_cat.unsqueeze(0)), dim=0)
            #<self.t_cat_time (B,T,Size) = (8,8,4)>
            self.t_cat_time = torch.cat((self.t_cat_time,t),dim=0)
            self.first_flag = True
    # <make dataset>
        print("train x =",self.x_cat_time.shape,x.device,"train t = " ,self.t_cat_time.shape,t.device)

        return self.x_cat_time,self.t_cat_time

    def cat_tensor(self, image_1_path, image_2_path, image_3_path, label_1_path, label_2_path,label_3_path):
        load_1_image_tensor = torch.load(image_1_path).to(self.device,non_blocking=True)
        load_2_image_tensor = torch.load(image_2_path).to(self.device,non_blocking=True)
        load_3_image_tensor = torch.load(image_3_path).to(self.device,non_blocking=True)

        load_1_label_tensor = torch.load(label_1_path).to(self.device,non_blocking=True)
        load_2_label_tensor = torch.load(label_2_path).to(self.device,non_blocking=True)
        load_3_label_tensor = torch.load(label_3_path).to(self.device,non_blocking=True)

        cat_image_tensor = torch.cat((load_1_image_tensor,load_2_image_tensor,load_3_image_tensor),dim=0)
        cat_label_tensor = torch.cat((load_1_label_tensor,load_2_label_tensor,load_3_label_tensor),dim=0)
        print("image tensor:",cat_image_tensor.shape)
        print("label tensor:",cat_label_tensor.shape)
        return cat_image_tensor,cat_label_tensor
    
    def cat_tensor_9(self, 
                 image_1_path, image_2_path, image_3_path,   
                 image_4_path, image_5_path, image_6_path, 
                 image_7_path, image_8_path, image_9_path, 
                 label_1_path, label_2_path, label_3_path, 
                 label_4_path, label_5_path, label_6_path, 
                 label_7_path, label_8_path, label_9_path):

        load_1_image_tensor = torch.load(image_1_path).to(self.device, non_blocking=True)
        load_2_image_tensor = torch.load(image_2_path).to(self.device, non_blocking=True)
        load_3_image_tensor = torch.load(image_3_path).to(self.device, non_blocking=True)
        load_4_image_tensor = torch.load(image_4_path).to(self.device, non_blocking=True)
        load_5_image_tensor = torch.load(image_5_path).to(self.device, non_blocking=True)
        load_6_image_tensor = torch.load(image_6_path).to(self.device, non_blocking=True)
        load_7_image_tensor = torch.load(image_7_path).to(self.device, non_blocking=True)
        load_8_image_tensor = torch.load(image_8_path).to(self.device, non_blocking=True)
        load_9_image_tensor = torch.load(image_9_path).to(self.device, non_blocking=True)

    # Load label tensors
        load_1_label_tensor = torch.load(label_1_path).to(self.device, non_blocking=True)
        load_2_label_tensor = torch.load(label_2_path).to(self.device, non_blocking=True)
        load_3_label_tensor = torch.load(label_3_path).to(self.device, non_blocking=True)
        load_4_label_tensor = torch.load(label_4_path).to(self.device, non_blocking=True)
        load_5_label_tensor = torch.load(label_5_path).to(self.device, non_blocking=True)
        load_6_label_tensor = torch.load(label_6_path).to(self.device, non_blocking=True)
        load_7_label_tensor = torch.load(label_7_path).to(self.device, non_blocking=True)
        load_8_label_tensor = torch.load(label_8_path).to(self.device, non_blocking=True)
        load_9_label_tensor = torch.load(label_9_path).to(self.device, non_blocking=True)

    # Concatenate image tensors
        cat_image_tensor = torch.cat((load_1_image_tensor, load_2_image_tensor, load_3_image_tensor, 
                                    load_4_image_tensor, load_5_image_tensor, load_6_image_tensor, 
                                    load_7_image_tensor, load_8_image_tensor, load_9_image_tensor), dim=0)
    
    # Concatenate label tensors
        cat_label_tensor = torch.cat((load_1_label_tensor, load_2_label_tensor, load_3_label_tensor, 
                                    load_4_label_tensor, load_5_label_tensor, load_6_label_tensor, 
                                    load_7_label_tensor, load_8_label_tensor, load_9_label_tensor), dim=0)

        print("Concatenated image tensor shape:", cat_image_tensor.shape)
        print("Concatenated label tensor shape:", cat_label_tensor.shape)
        return cat_image_tensor, cat_label_tensor
    
    def load_path_tensor(self,image_file_path,label_file_path):
        print("image_path:",image_file_path)
        print("label_path:",label_file_path)
        image_path_list = glob.glob(image_file_path)
        label_path_list = glob.glob(label_file_path)
        print("image_path_list:",len(image_path_list))
        print("label_path_list:",len(label_path_list))
        first_flag_image = True
        first_flag_label = True

        cat_image_tensor = None
        cat_label_tensor = None
        image_path_list_size = len(image_file_path)
        for i in range(image_path_list_size):
            if first_flag_image:
                # print("current label",label_path_list[i]+'image.pt')
                image_file_path_0 = image_path_list[i]+'/image.pt'
                image_file_path_1 = image_path_list[i+1]+'/image.pt'
                print("current image",image_file_path_0)
                load_tensor_0 = torch.load(image_file_path_0)
                print("tensor size:",load_tensor_0.shape)
                print("current image_1",image_file_path_1)
                load_tensor_1 = torch.load(image_file_path_1)
                print("tensor size:",load_tensor_1.shape)
                # cat_image_tensor = torch.cat((torch.load(image_path_list[i]+'/image.pt'),torch.load(image_file_path[i+1]+'/image.pt')))
                cat_image_tensor = torch.cat((load_tensor_0,load_tensor_1),dim=0)

                label_file_path_0 = label_path_list[i]+'/label.pt'
                label_file_path_1 = label_path_list[i+1]+'/label.pt'
                print("current label_0:",label_file_path_0)

                load_label_0 = torch.load(label_file_path_0)
                print("tensor size:",load_label_0.shape)
                print("current label_1:",label_file_path_1)
                load_label_1 = torch.load(label_file_path_1)
                print("tensor size:",load_label_1.shape)

                # cat_label_tensor = torch.cat(torch.load((label_path_list[i]+'/label.pt'),torch.load(label_file_path[i+1]+'/label.pt')))
                cat_label_tensor = torch.cat((load_label_0,load_label_1) ,dim=0)

                first_flag_image = False
            else:
                print("current image",label_path_list[i]+'image.pt')
                # cat_image_tensor = torch.cat((cat_image_tensor,torch.load(image_file_path[i+1]+'/image.pt')),dim=1)
                image_file_path_current = image_path_list[i]+'/image.pt'
                current_load_tensor = torch.load(image_file_path_current)
                cat_image_tensor = torch.cat((cat_image_tensor,current_load_tensor),dim=0)
                print("tensor size:",current_load_tensor.shape)

                label_path_current = label_path_list[i]+'/label.pt'
                print("current label",label_path_current)
                load_label_current = torch.load(label_path_current)
                print("torch size:",load_label_current.shape)
                # cat_label_tensor = torch.cat((cat_label_tensor,torch.load(label_file_path[i+1]+'/label.pt')),dim=1)
                cat_label_tensor = torch.cat((cat_label_tensor,load_label_current),dim=0)
            if i==len(image_path_list)-1:
                break
        label_path_list_size = len(label_path_list)
        # for j in range(label_path_list_size):
        #     if first_flag_label:
        #         label_file_path_0 = label_path_list[j]+'/label.pt'
        #         label_file_path_1 = label_path_list[j+1]+'/label.pt'
        #         print("current label_0:",label_file_path_0)
        #         load_label_0 = torch.load(label_file_path_0)
        #         print("tensor size:",load_label_0.shape)
        #         print("current label_1:",label_file_path_1)
        #         load_label_1 = torch.load(label_file_path_1)
        #         print("tensor size:",load_label_1.shape)

        #         # cat_label_tensor = torch.cat(torch.load((label_path_list[j]+'/label.pt'),torch.load(label_file_path[j+1]+'/label.pt')))
        #         cat_label_tensor = torch.cat((torch.load(label_file_path_0),torch.load(label_file_path_1)) ,dim=0)
        #         first_flag_label = False
        #     else:
        #         label_path_current = label_path_list[j]+'/label.pt'
        #         print("current label_0:",label_path_current)
        #         load_label_current = torch.load(label_path_current)
        #         print("torch size:",load_label_current.shape)
        #         # cat_label_tensor = torch.cat((cat_label_tensor,torch.load(label_file_path[j+1]+'/label.pt')),dim=1)
        #         cat_label_tensor = torch.cat((cat_label_tensor,torch.load(label_path_current)),dim=0)
        #     if j>=len(cat_label_tensor)-1:
        #         break
        print("cat_image_tensor:",cat_image_tensor.shape)
        print("cat_label_tensor:",cat_label_tensor.shape)

        return cat_image_tensor, cat_label_tensor
    def tensor_info(self, load_x_tensor, load_t_tensor):
        print(self.device)
        # load_x_tensor = torch.load(load_x_tensor)
        # load_t_tensor = torch.load(load_t_tensor)
        print("x_tensor:", load_x_tensor.shape, "t_tensor:", load_t_tensor.shape)
        print("label info :", torch.sum(load_t_tensor ,dim=0))
        
    def cat_training(self, load_x_tensor, load_t_tensor, load_flag):
        # self.device = torch.device('cuda')
        print(self.device)
        if load_flag:
            load_x_tensor = torch.load(load_x_tensor)
            load_t_tensor = torch.load(load_t_tensor)
        print("x_tensor:", load_x_tensor.shape, "t_tensor:",load_t_tensor.shape)
        print("label info :", torch.sum(load_t_tensor, dim=0))
        dataset = TensorDataset(load_x_tensor, load_t_tensor)
        train_dataset = DataLoader(
            dataset, batch_size=BATCH_SIZE, generator=torch.Generator('cpu'), shuffle=True)
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
                # nn.utils.clip_grad_value_(parameters=loss_all.,clip_value=CLIP_VALUE)
                self.train_accuracy += torch.sum(torch.max(y_train, 1)
                                                    [1] == torch.max(t_label_train, 1)[1]).item()
                print("epoch:",epoch, "accuracy :", self.train_accuracy, "/",len(t_label_train),
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
            epoch_accuracy = batch_accuracy /len(train_dataset)
            print("epoch loss:",epoch_loss,"epoch accuracy:",epoch_accuracy)
            # self.writer.add_scalar("epoch loss", epoch_loss, epoch)
            # self.writer.add_scalar("epoch accuracy",epoch_accuracy,epoch)
        print("Finish learning!!")
        finish_flag = True

        return self.train_accuracy, self.loss_all
    def training(self, image_path,label_path):
        # self.device = torch.device('cuda')
        print(self.device)
        load_x_tensor = torch.load(image_path)
        load_t_tensor = torch.load(label_path)
        # print("x_tensor:",load_x_tensor,"t_tensor:",load_t_tensor)
        print("label info :",torch.sum(load_t_tensor ,dim=0))
        dataset = TensorDataset(load_x_tensor, load_t_tensor)
        train_dataset = DataLoader(
            dataset, batch_size=BATCH_SIZE, generator=torch.Generator('cpu'), shuffle=False)
    # <training mode>
        self.net.train()
        self.train_accuracy = 0
    
    # <split dataset and to device>
        for epoch in range(EPOCH_NUM):
            print('epoch', epoch) 
            for x_train,t_label_train in train_dataset:
                # if i == random_train:
                # x_train,t_label_train = train_dataset
                # x_train.to(self.device, non_blocking=True)
                # t_label_train.to(self.device, non_blocking=True)
                x_train = x_train.to(self.device,non_blocking=True)
                t_label_train = t_label_train.to(self.device, non_blocking=True)
        # <use transform>
                # print("ddd=",x_train[0,:,:,:,:].shape)
            # for i in range(BATCH_SIZE):
                # x_train = self.transform_color(x_train)
                # self.transform_train(x_train[i,:,:,:,:])
                # x_train =self.normalization(x_train)
        # <learning>
                self.optimizer.zero_grad()
                y_train = self.net(x_train)
                loss_all = self.criterion(y_train, t_label_train)
                # print("y = ",y_train.shape,"t=",t_label_train)
                self.train_accuracy += torch.sum(torch.max(y_train, 1)
                                                    [1] == torch.max(t_label_train, 1)[1]).item()
                print("epoch:",epoch, "accuracy :", self.train_accuracy, "/",len(t_label_train),
                        (self.train_accuracy/len(t_label_train))*100, "%","loss :",loss_all.item())
                self.writer.add_scalar("loss", loss_all.item(), self.count)
                self.writer.add_scalar("accuracy",(self.train_accuracy/len(t_label_train))*100,self.count)
                loss_all.backward()
                self.optimizer.step()
                self.loss_all = loss_all.item()
                self.count += 1
                self.train_accuracy = 0
        print("Finish learning!!")
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
            # self.x_cat_time_test = self.x_cat_test.unsqueeze(0)
            # torch.cat((self.x_cat_time, self.x_cat_test.unsqueeze(0)), dim=0)
            #self.first_test_flag = True
            self.intersection_test = self.net(self.x_cat_test.unsqueeze(0))
            print("s:",self.intersection_test.shape)
            self.x_cat_test = self.x_cat_test[1:]
        # print(x_test_ten.shape,x_test_ten.device,c_test.shape,c_test.device)
    # <test phase>
            # self.intersection_test = self.net(self.x_cat_test.unsqueeze(0))
        
        return torch.max(self.intersection_test, 1)[1].item()

    def save_tensor(self, dataset_tensor, save_path, file_name):
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save(dataset_tensor, path+file_name)
        print("save_path: ",save_path + file_name)

    def save(self, save_path):
        # <model save>
        path = save_path + time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(path)
        torch.save(self.net.state_dict(), path + '/model_gpu.pt')
    
    def load(self, load_path):
        # <model load>
        self.net.load_state_dict(torch.load(load_path))
        print(load_path)
if __name__ == '__main__':
    dl = deep_learning()
