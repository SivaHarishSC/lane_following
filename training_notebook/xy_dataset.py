import torch
import os
import glob
import uuid
import PIL.Image
import torch.utils.data
import subprocess
import cv2
import numpy as np

class XYDataset(torch.utils.data.Dataset):
    def __init__(self, directory, transform=None, random_hflip=False):
        super(XYDataset, self).__init__()
        self.directory = directory
        self.transform = transform
        self.random_hflip = random_hflip
        self.refresh()
        
    def __len__(self):
        return len(self.annotations)
    
    def __getitem__(self, idx):
        ann = self.annotations[idx]
        image = cv2.imread(ann['image_path'], cv2.IMREAD_COLOR)
        image = PIL.Image.fromarray(image)
        if self.transform is not None:
            image = self.transform(image)
        
        angle = ann['angle']  # Get the steering angle

        if self.random_hflip and float(np.random.random(1)) > 0.5:
            image = torch.from_numpy(image.numpy()[..., ::-1].copy())
            angle = -angle  # Flip the steering angle if applicable
            
        angle_class = self.angle_to_class(angle)  # Convert angle to class index
        return image, 0, angle_class
    
    def _parse(self, path):
        basename = os.path.basename(path)
        items = basename.split('_')
        angle = int(items[0])  # Assuming filename format is '<angle>_<uuid>.jpg'
        return angle
        
    def refresh(self):
        self.annotations = []
        for image_path in glob.glob(os.path.join(self.directory, '*.jpg')):
            angle = self._parse(image_path)
            self.annotations += [{
                'image_path': image_path,
                'angle': angle
            }]
        
    def save_entry(self, image, angle):
        filename = '%d_%s.jpg' % (angle, str(uuid.uuid1()))
        image_path = os.path.join(self.directory, filename)
        cv2.imwrite(image_path, image)
        self.refresh()

    def angle_to_class(self, angle):
        """Converts a steering angle to a class index"""
        if angle == -28:
            return 0
        elif angle == 0:
            return 1
        elif angle == 28:
            return 2
        else:
            raise ValueError("Unexpected angle value")