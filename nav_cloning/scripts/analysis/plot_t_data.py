import torch
import roslib
import matplotlib.pyplot as plt

# データセットのパスを設定
mode = "selected_training"
place = "cit3f"
name = "real"

# image_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(mode) + '/' + str(place) + '/' + 'route_8' + '/1/' +  '/image' + '/image.pt'
# dir_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(mode) + '/' + str(place) + '/' + 'route_8' + '/1/' + '/dir' + '/dir.pt'
# vel_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset_with_dir_' + str(mode) + '/' + str(place) + '/' + 'route_8' + '/1/' + '/vel' + '/vel.pt'

image_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset/image/' + str(name) + '/image.pt'
dir_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset/dir/' + str(name) + '/dir.pt'
vel_path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/dataset/vel/' + str(name) + '/vel.pt'
# データセットを読み込む関数
def load_dataset(image_path, dir_path, vel_path):
    # データの読み込み
    x_tensor = torch.load(image_path)
    c_tensor = torch.load(dir_path)
    t_tensor = torch.load(vel_path)

    # データの形状を表示
    print("load_image:", x_tensor.shape)
    print("load_dir:", c_tensor.shape)
    print("load_vel:", t_tensor.shape)

    # t_tensorがGPU上にある場合、CPUに移動してnumpyに変換
    if t_tensor.is_cuda:
        t_tensor = t_tensor.cpu()
    if c_tensor.is_cuda:
        c_tensor = c_tensor.cpu()

    # numpy配列に変換
    t_tensor = t_tensor.numpy().squeeze()
    c_tensor = c_tensor.numpy()

    return x_tensor, c_tensor, t_tensor

def plot_angular_velocity(c_tensor, t_tensor):
    """
    目標方向 (c_tensor) ごとに角速度 (t_tensor) の分布をプロットします。
    """
    directions = ['Forward', 'Left', 'Right']
    for i, direction in enumerate(directions):
        # ワンホットベクトルに基づいてマスクを作成
        mask = (c_tensor[:, i] == 1)
        filtered_t_tensor = t_tensor[mask]

        # ヒストグラムプロット
        plt.figure(figsize=(10, 6))
        plt.hist(filtered_t_tensor, bins=50, edgecolor='black')
        plt.xlabel("Angular Velocity (t_tensor)")
        plt.ylabel("Number of Samples")
        plt.title(f"Distribution of Angular Velocity (t_tensor) - {direction}")
        plt.grid()
        plt.show()


# データセットの読み込みとプロットの実行例
x_tensor, c_tensor, t_tensor = load_dataset(image_path, dir_path, vel_path)
plot_angular_velocity(c_tensor, t_tensor)
