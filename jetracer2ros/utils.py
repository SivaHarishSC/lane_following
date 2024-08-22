import torch
import torchvision.transforms as transforms
import torch.nn.functional as F
import cv2
import PIL.Image
import numpy as np

# Define the same mean and std used during training
mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

# Define the transforms as used during training
TRANSFORMS = transforms.Compose([
    transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),  # Note: If we don't want color jittering during inference, we can remove the ColorJitter transform from the TRANSFORMS.
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

def preprocess(image):
    device = torch.device('cuda')
    image = PIL.Image.fromarray(image)
    image = TRANSFORMS(image).to(device)
    return image[None, ...]
