from resnet18 import ResNet18MNIST
import torch,os,cv2
import numpy as np


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = ResNet18MNIST()
checkpoint = torch.load('1/trained_model/ACC_97.39.pth')
model.load_state_dict(checkpoint)
net = model.to(device)

image_root_path = "1/sonar_data/train/img"
image_lst = [i for i in os.listdir(image_root_path) if i.endswith(".jpg")]
#print(type(image_lst))


with torch.no_grad():
    while True:
    #for image_name in image_lst[100:300]:
        #image_path = os.path.join(image_root_path, image_name)
        
        #image = cv2.imread(image_path).astype(np.float32)
        
        image = queue.get()
        image = cv2.resize(image, (480, 480))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32)
        image = image / 255.0 - 0.5
        image = np.transpose(image, (2, 0, 1))
        image = torch.Tensor(image)
        
        image = image.to(device)

        out = net(image.unsqueeze(0))
        pred = out.argmax(dim=1)
        print(f"图片{image_name}所属的label分别是:",pred.item())
        
    
    