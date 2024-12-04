import numpy as n
from PIL import Image

def IoU(img1_path, img2_path):
    
    # Convert imgs to binaries
    img1 = Image.open(img1_path).convert('1')  
    img2 = Image.open(img2_path).convert('1')
    img1 = n.array(img1)
    img2 = n.array(img2)
    
    # Intersection = AND both image pixel arrays 
    intersection = n.sum((img1 == 1) & (img2 == 1))  

    # Union = OR both image pixel arrays 
    union = n.sum((img1 == 1) | (img2 == 1)) 
    
    # Calculate IoU
    iou = intersection / union if union != 0 else 0  # Avoid division by zero
    
    return iou


if __name__ == "__main__": 
    
    img1_path = '../../GT_stencil_for_Validation.png'
    img2_path = '../../2D_Actual_Traj_for_Validation.png'
    iou = IoU(img1_path, img2_path)
    print("Intersection over Union (IoU) result: ", iou)
    print("Validation result: ", round(iou*100,2), " % match between Ground Truth and Robot Arm Sketch")
