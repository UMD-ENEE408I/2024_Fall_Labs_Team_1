import logging
import threading
import time
import cv2
import queue
import torch
import torchvision
import torchvision.transforms as transforms
import torch.nn.functional as F
import client_server
import PIL.Image

mean = torch.Tensor([0.485, 0.456, 0.406]).cpu()
std = torch.Tensor([0.229, 0.224, 0.225]).cpu()

task_queue = queue.Queue()
model = torchvision.models.resnet18(weights='IMAGENET1K_V1')

def preprocess(image):
    device = torch.device('cpu')
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]

def enqueue(message):
    #task_queue.put(message)
    logging.info(f'Queued an image request message: {message}')

def processor():
    global model
    while True:
        msg = task_queue.get(block=True, timeout=None)
        logging.info(f'Popped queue, got message: {msg}')

        cam = cv2.VideoCapture('/dev/video0')
        result, image = cam.read()

        if result:
            use_model = ('colors.pth' if msg['model'] == 'colors' else 'crypto.pth')
            dim = (3 if msg['model'] == 'colors' else 5)

            model.fc = torch.nn.Linear(512, dim)
            model = model.to(torch.device('cpu'))

            logging.debug(f"Using model {use_model}...")
            model.load_state_dict(torch.load(use_model, map_location=torch.device('cpu'), weights_only=True))

            model.eval()
            output = model(image)
            output = F.softmax(output, dim=1).detach().cpu().numpy().flatten()

            client_server.send_to_client(msg['name'], {'image_result': output.argmax()})
        else:
            client_server.send_to_client(msg['name'], {'image_result': 'FAILED, cv2'})


def start_handler_thread():
    threading.Thread(name='Image Processing Queue Handler', args=(), target=processor).start()
