import logging
import threading
import numpy as np
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
model = torchvision.models.resnet50(weights='IMAGENET1K_V1')

red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)

green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)

blue_lower = np.array([94, 80, 2], np.uint8)
blue_upper = np.array([120, 255, 255], np.uint8)

def preprocess(image):
    device = torch.device('cpu')
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]

def enqueue(message):
    task_queue.put(message)
    logging.info(f'Queued an image request message: {message}')

def processor():
    global model
    while True:
        msg = task_queue.get(block=True, timeout=None)
        logging.info(f'Popped queue, got message: {msg}')

        #cap = cv2.VideoCapture('/dev/video0')
        #cap = cv2.VideoCapture('http://' + client_server.get_ip_from_name('espcam') + "/cam.jpg")
        cap = cv2.VideoCapture('http://192.168.1.101:81/stream')

        logging.info('Got image!')

        if cap.isOpened():
            result, image = cap.read()
            cv2.imwrite('img_curr.jpg', image)
        else:
            raise Exception("Could not get image from WiFiCam (cv2)")

        result_to_send = None

        if result:
            if msg['model'] == 'crypto':
                use_model = 'crypto.pth'
                dim = 5

                model.fc = torch.nn.Linear(512, dim)
                model = model.to(torch.device('cpu'))

                logging.debug(f"Using model {use_model}...")
                model.load_state_dict(torch.load(use_model, map_location=torch.device('cpu'), weights_only=True))

                model.eval()
                output = model(image)
                output = F.softmax(output, dim=1).detach().cpu().numpy().flatten()
                result_to_send = output.argmax()
            else:
                hsvFrame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
                green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
                blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

                kernel = np.ones((5, 5), "uint8")
                # For red color
                red_mask = cv2.dilate(red_mask, kernel)
                res_red = cv2.bitwise_and(image, image, mask=red_mask)

                # For green color
                green_mask = cv2.dilate(green_mask, kernel)
                res_green = cv2.bitwise_and(image, image, mask=green_mask)

                # For blue color
                blue_mask = cv2.dilate(blue_mask, kernel)
                res_blue = cv2.bitwise_and(image, image, mask=blue_mask)

                red_count = cv2.countNonZero(cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY))
                green_count = cv2.countNonZero(cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY))
                blue_count = cv2.countNonZero(cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY))

                logging.info(f'Got the following counts: R: {red_count}, G: {green_count}, B: {blue_count}')

                maxi = max(red_count, green_count, blue_count)
                if (maxi == red_count):
                    result_to_send = 'RED'
                elif (maxi == green_count):
                    result_to_send = 'GREEN'
                elif (maxi == blue_count):
                    result_to_send = 'BLUE'

            if result_to_send:
                    client_server.send_to_client({'name': msg['name'], 'image_result': result_to_send})
            else:
                logging.info(f'Failed to send result - is None')
        else:
            client_server.send_to_client(msg['name'], {'image_result': 'FAILED, cv2'})


def start_handler_thread():
    threading.Thread(name='Image Processing Queue Handler', args=(), target=processor).start()
