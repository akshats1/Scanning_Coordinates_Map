from flask import Flask, render_template, Response,jsonify,request,send_file
import time
import io
import threading
from PIL import Image
import PIL.Image
from camera_update_nn5 import Camera # 7_July
import atexit
from datetime import datetime
import serial
import os
import boto3
import os
from werkzeug.utils import secure_filename
import shutil
import logging
import signal
import sys
import time
from threading import Thread
import numpy as np
import cv2
from time import sleep
#from picamera import PiCamera

#camera = Camera(save_dir=save_dir)
cur_time = datetime.now()
dir_path = f"/media/pi/3332-3561/scan_{cur_time.strftime('%Y%m%d_%H%M')}/"
save_dir=dir_path
camera = Camera(save_dir=save_dir)
############## declaring x,y,z axis globally##########
x_axis=0
y_axis=0
z_axis=0

class Microscope:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600, timeout=1):
        self.board = serial.Serial(port, baud_rate, timeout=timeout)
      
        self.x = 0
        self.y = 0
        self.z = 0
        self.scan_count = 0
        #atexit.register(self.close)  # Ensure resources are released at exit

    def movexclock(self, distance):
        self.board.write(f"xclk,{distance}".encode())
        self._wait_for_completion()
        self.x += distance

    def movexanticlock(self, distance):
        self.board.write(f"xcclk,{distance}".encode())
        self._wait_for_completion()
        self.x -= distance

    def movey(self, steps):
        self.board.write(f"ycclk,{steps}".encode())
        self._wait_for_completion()
        self.y += steps

    def moveyc(self, steps):
        """
        command = f"yclk,{steps}"
        self.board.write(command.encode("utf-8"))
        self.y -= steps
        sleep(1)
        """
        self.board.write("yclk,{}".format(steps).encode())
        self._wait_for_completion()
        self.y -= steps

    def movezclock(self, distance):
        self.board.write(f"zclk,{distance}".encode())
        self._wait_for_completion()
        self.z -= distance

    def movezanticlock(self, distance):
        self.board.write(f"zcclk,{distance}".encode())
        self._wait_for_completion()
        self.z += distance
    def init(self):
        self.board.write("init".encode())
    def home(self, x_axis, y_axis, z_axis):
        print("Home Position for x, y, z in Process")
        self.movexclock(x_axis)
        sleep(4)
        self.movey(y_axis)
        sleep(5)
        self.movezanticlock(z_axis)
        sleep(8)
    
    def calculate_variance(self, image_bytes):
        image = Image.open(image_bytes)
        image = image.convert("L")
        image_array = np.array(image)
        variance = np.var(image_array)
        return variance
    
    def preprocess_image(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply a median filter to reduce noise
        filtered = cv2.medianBlur(gray, 5)
        return filtered

    def variance(self, image):
        bg = cv2.GaussianBlur(image, (11, 11), 0)
        v = cv2.Laplacian(bg, cv2.CV_64F).var()
        return v
    
    def roixy_10x(self, x, y):
        self.movexclock(x)
        sleep(8)
        self.movey(y)
        global x_axis, y_axis, z_axis
        x_axis += x
        y_axis += y
        print("Movement of Steps")
        print("x_axis", x_axis)
        print("y_axis", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis", str(x_axis * 0.075) + " mm")
        print("y_axis", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)

    def _wait_for_completion(self):
        while True:
            data = self.board.readline()
            if data == b'Done\r\n':
                break
        self.board.reset_input_buffer()

    def capture_image(self):
        stream = io.BytesIO()
        try:
            camera.capture(stream, format='jpeg')
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None
        
        stream.seek(0)
        image_data = stream.read()
        
        if not image_data:
            print("Error: Stream is empty after capture")
            return None
        
        image = np.frombuffer(image_data, dtype=np.uint8)
        if image.size == 0:
            print("Error: Image buffer is empty")
            return None
        
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        if image is None:
            print("Error: Failed to decode image")
            return None
        
        return image
        """
        stream = io.BytesIO()
        #self.camera2.capture(stream, format='jpeg')
        camera.capture_image()
        
        stream.seek(0)
        image = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        return image
        """
    
    ## Autofocus Function
    def auto(self):
        obj_value = 10
        z_positions = []
        variances = []
        max_variance = 0
        max_z = self.z

        #self.camera.start_preview()
        self.board.reset_input_buffer()
        sleep(1)

        camera.resolution = (640,480) #we can have increase the resolution 640X480
        camera.framerate = 24

        if obj_value == 4:
            step_size = 50
        else:
            step_size = 35 # 35

        direction = 1  # 1 for upward, -1 for downward
        for i in range(41):
           
            image_buff=camera.capture_image() #8_August
            if image_buff is None:
                print(f"Error capturing image at iteration {i}")
                continue  # Skip this iteration if capture failed
            #stream.seek(0)
            
           # image = np.frombuffer(image_buff, dtype=np.uint8)
            #Simage = cv2.imdecode(image, cv2.IMREAD_COLOR)
            
            #print("Image return ",image_buff)
            
            preprocessed_image = self.preprocess_image(image_buff)
            current_variance = self.variance(image_buff) # 9_Aug #removing noise
            #current_variance = self.variance(image)
            variances.append(current_variance)
            z_positions.append(self.z)

            print(f"Iteration {i+1}: x_axis={self.x}, y_axis={self.y}, z_axis={self.z}")

            if current_variance > max_variance:
                max_variance = current_variance
                max_z = self.z

            if i < 40:
                if current_variance >= max_variance:
                    if direction == 1:
                        self.movezanticlock(step_size)
                    else:
                        self.movezclock(step_size)
                else:
                    if direction == 1:
                        direction = -1
                        self.movezclock(step_size)
                    else:
                        break

        # Adjust to the position with the maximum variance
        adjust_steps = self.z - max_z
        if adjust_steps > 0:
            self.movezclock(adjust_steps)
        else:
            self.movezanticlock(abs(adjust_steps))

    
        sleep(5)
        print(variances)
        print(z_positions)
        print(max_z)
        print(max_variance)
    

    def scan(self):
        cur_time = datetime.now()
        dir_path = f"/media/pi/3332-3561/scan_{cur_time.strftime('%Y%m%d_%H%M')}/"
       # os.makedirs(dir_path, exist_ok=True)
        self.auto()

        for i in range(25):
            for j in range(28):
                #self.capture_and_save_image(dir_path, i, j)
                if j%2==0:
                    self.auto()
                if i % 2 == 0:
                    sleep(2)
                    self.capture_and_save_image(dir_path, i, j)
                    self.movexanticlock(8)
                    
                else:
                    sleep(2)
                    self.capture_and_save_image(dir_path, i, 27 -j)
                    self.movexclock(8)
                    
                print(f"Iteration {i * 28 + j + 1}: x_axis={self.x}, y_axis={self.y}, z_axis={self.z}")
            #sleep(2)
            self.moveyc(8)  # move yclk for scanning

    def capture_and_save_image(self, dir_path, i, j):
        # Ensure the directory exists
        os.makedirs(dir_path, exist_ok=True)
        
        
        camera.resolution=camera.pic_res
        image = camera.capture_image_new()
        if image is None:
            print(f"Image capture failed at row {i}, column {j}")
            return
        
        
        
        image_path = f"{dir_path}imagerow{i},{j}.tiff"
        image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        image_pil.save(image_path, format='TIFF')
        sleep(6)
        self.scan_count += 1

   

# Initialize the Microscope class
microscope = Microscope()





# 16_july
#from autofocus_Update_new_3 import Autofocus

#17_july
#camera_lock=threading.Lock()


#mtr=Motor()
try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600)
except Exception as e:
    print(f"Error opening serial port: {e}")


app = Flask(__name__)
# adding save directory

'''
*****************Motor Control Routes Start************ 
'''






def video_feed_2():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
# 16 May 
def start_page():
    #return render_template('index.html')
    #return render_template('index.html')
    return render_template('index.html')
"""    
@app.route('/index')
def index_page():
    return render_template('index.html')
"""
    
#gallery file with aws need to integrate over here





def generate_frames():
    pass
    ## 16_july_starts
    #af.init_camera()
    #while True:
        #with af.lock:
            #if af.camera:
             #   stream=io.BytesIO()
            #    af.camera.capture(stream, format='jpeg')
           #     stream.seek(0)
          #      frame=stream.read()
         #       yield(b'--frame\r\n'
        #              b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
       #time.sleep(0.5) 
    ##16_july_Ends
    
    
    #### original code Starts
    while True:
        frame = camera.get_frame()  # Get a frame from the camera
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.5)  # Adjust sleep time as neede
#####original code ends

@app.route('/video_record')
def video_record():
    filename=datetime.now().strftime("%Y-%m-%d_%H-%M-%S")+'.mp4'
    # for recording the live video
    camera.start_recording(filename)
    return 'video recording started'

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')



"""
@app.route('/capture')
def capture():
    camera.capture_image()  # Capture an image
    #return 'Image captured!'
    return 'Image Captured'

@app.route('/stop_record')
def stop_capture(): 
    camera.stop_recording()  # Stop video recording
    return 'Video recording stopped!'
    
    
"""
def cleanup():
    
    camera.close()# After Picamera stop preview close the camera

@app.route('/scan', methods=['POST'])
def scan():
    print("Initialization Starting Please Wait......")
    xi = 350
    yi = 200
    sleep(3)
    
    microscope.init()
    sleep(25) # 21
    
    print("Initialization Stops for x, y, z in Process")
    
    x_axis = 0
    y_axis = 0
    z_axis = 0
    print("x_axis", x_axis)
    print("y_axis", y_axis)
    print("z_axis", z_axis)
    x_axis = 30
    y_axis = 100
    z_axis = 4000
    microscope.home(x_axis, y_axis, z_axis)
    print("Movement of Steps After Home Position")
    print("x_axis", x_axis)
    print("y_axis", y_axis)
    print("z_axis", z_axis)
    print("Movement of Steps in mm")
    print("x_axis", str(x_axis * 0.075) + " mm")
    print("y_axis", str(y_axis * 0.075) + " mm")
    print("z_axis", z_axis)
    
    print("10X Objective value 10 Selected")
    in_obj = 10
    
    if in_obj == 4:
        if z_axis > 3500:
            ans_z = z_axis - 3500
            z_axis = z_axis - ans_z
            microscope.movezclock(ans_z)
        else:
            ans_z = 3500 - z_axis
            z_axis = z_axis + ans_z
            microscope.movezanticlock(ans_z)
        
        x = 80
        y = 0
        microscope.roixy_4x(x, y)
        sleep(2)
    elif in_obj == 10:
        if z_axis > 13550: # 13350 just for now 13550 for scanning 
            ans_z = z_axis - 13550
            z_axis = z_axis - ans_z
            microscope.movezclock(ans_z)
        else:
            ans_z = 13550 - z_axis
            z_axis = z_axis + ans_z
            microscope.movezanticlock(ans_z)
        sleep(6)
        
        print("Movement of Steps for Starting Position")
        print("x_axis", x_axis)
        print("y_axis", y_axis)
        print("z_axis", z_axis)
        print("Movement of Steps in mm")
        print("x_axis", str(x_axis * 0.075) + " mm")
        print("y_axis", str(y_axis * 0.075) + " mm")
        print("z_axis", z_axis)
        
        x = 155 #175
        y = 120 # 115 # 125
        microscope.roixy_10x(x, y)## moving to Starting Position 
        sleep(4)
    elif in_obj == 40:
        microscope.movezanticlock(16650)
        sleep(22)
    def run_scan():
        
        
        microscope.scan()
    threading.Thread(target=run_scan).start()
    return jsonify({"message": "Scanning started"})





if __name__ == "__main__":
    atexit.register(cleanup)# for releasing the resources of picamera
    app.run(debug=True,use_reloader=False)

