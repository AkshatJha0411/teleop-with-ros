import tkinter as tk
from tkinter import PhotoImage, DoubleVar, IntVar, HORIZONTAL, DOTBOX
from geometry_msgs.msg import Twist 
import rospy, time
import coordinate_processs as cp

# Initialize the ROS node
rospy.init_node('Teleop_UI')

# Setting data rate as 1
rate = rospy.Rate(1)
movement_array = []

# Publishing to turtle1/cmd_vel
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

# Functions for robot movement
def pub_forward(event=None):
    data = Twist()
    data.linear.x = 2
    data.angular.z = 0
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()
    movement_array.append("F")

def pub_backward(event=None):
    data = Twist()
    data.linear.x = -2
    data.angular.z = 0
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()
    movement_array.append("B")

def pub_left(event=None):
    data = Twist()
    data.linear.x = 0
    data.angular.z = 2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()
    movement_array.append("L")

def pub_right(event=None):
    data = Twist()
    data.linear.x = 0
    data.angular.z = -2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    movement_array.append("R")

def pub_stop(event=None):
    data = Twist()
    data.linear.x = 2
    data.angular.z = 0
    data.angular.y = 0
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()

def change_light_mode(event=None):
    check_light.set(0 if check_light.get() == 1 else 1)

def diff_fleft(event=None):
    data = Twist()
    data.linear.x = 2
    data.angular.z = 2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()

def diff_fright(event=None):
    data = Twist()
    data.linear.x = 2
    data.angular.z = -2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()

def diff_bleft(event=None):
    data = Twist()
    data.linear.x = -2
    data.angular.z = 2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()

def diff_bright(event=None):
    data = Twist()
    data.linear.x = -2
    data.angular.z = -2
    data.angular.y = int(var.get())
    data.angular.x = check_light.get()
    pub.publish(data)
    rate.sleep()

def make_triangle(event=None):
    pub_forward()
    time.sleep(0.3)
    pub_right()
    time.sleep(0.66)
    pub_forward()
    time.sleep(0.3)
    pub_right()
    time.sleep(0.66)
    pub_forward()
    time.sleep(0.3)
    pub_right()
    time.sleep(0.66)
    pub_forward()
    time.sleep(0.3)
    pub_stop()

def turnoff(event=None):
    root.destroy()
    final_coordinates = cp.main(movement_array)
    print(final_coordinates)

def return_to_owner_mode(event=None):
    final_coordinates = cp.main(movement_array)
    print(final_coordinates)

# Create the main window
root = tk.Tk()
root.bind('w', pub_forward)
root.bind('s', pub_backward)
root.bind('d', pub_right)
root.bind('a', pub_left)
root.bind('e', pub_stop)
root.bind('l', change_light_mode)
root.bind('q', turnoff)
var = DoubleVar()
check_light = IntVar()
root.geometry('+550+150')
root.title("Teleoperated robot using ROS")

# Importing necessary images for UI
fimg = PhotoImage(file='Forward.png').subsample(3, 3)
bimg = PhotoImage(file='Backwards.png').subsample(3, 3)
rimg = PhotoImage(file='right.png').subsample(3, 3)
limg = PhotoImage(file='left_turn.png').subsample(3, 3)
simg = PhotoImage(file='stop.png').subsample(3, 3)
drimg = PhotoImage(file='dright.png').subsample(3, 3)
dlimg = PhotoImage(file='dleft.png').subsample(3, 3)
bdrimg = PhotoImage(file='bdright.png').subsample(3, 3)
bdlimg = PhotoImage(file='bdleft.png').subsample(3, 3)

# Defining and placing different components of UI
label1 = tk.Label(root, text="Robot ROS Control")
label1.grid(row=0, column=1, pady=10)

buttonf = tk.Button(root, text="Forward", command=pub_forward, image=fimg)
buttonf.grid(row=1, column=1, padx=5, pady=5)

buttonb = tk.Button(root, text="Backward", command=pub_backward, image=bimg)
buttonb.grid(row=3, column=1, padx=5, pady=5)

buttonl = tk.Button(root, text="Left", command=pub_left, image=limg)
buttonl.grid(row=2, column=0, padx=5, pady=5)

buttonr = tk.Button(root, text="Right", command=pub_right, image=rimg)
buttonr.grid(row=2, column=2, padx=5, pady=5)

buttondfl = tk.Button(root, text="Curve Forward Left", command=diff_fleft, image=dlimg)
buttondfl.grid(row=1, column=0, padx=5, pady=5)

buttondfr = tk.Button(root, text="Curve Forward Right", command=diff_fright, image=drimg)
buttondfr.grid(row=1, column=2, padx=5, pady=5)

buttondbl = tk.Button(root, text="Curve Backward Left", command=diff_bleft, image=bdlimg)
buttondbl.grid(row=3, column=0, padx=5, pady=5)

buttondbr = tk.Button(root, text="Curve Backward Right", command=diff_bright, image=bdrimg)
buttondbr.grid(row=3, column=2, padx=5, pady=5)

buttonstop = tk.Button(root, text="Stop!", command=pub_stop, image=simg)
buttonstop.grid(row=2, column=1, padx=5, pady=5)

# Adjust the column and row weights so the layout expands nicely
for i in range(4):
    root.grid_rowconfigure(i, weight=1)
    root.grid_columnconfigure(i, weight=1)

# Adding padding to the speed text and its scale
speed_scale = tk.Scale(root, from_=3, variable=var, to=7, length=200, label="Speed", orient=HORIZONTAL, cursor=DOTBOX)
speed_scale.grid(row=4, column=0, columnspan=3, pady=10)  # Added padding to the top

traingle_but = tk.Button(root, text="Triangle", command=make_triangle)
traingle_but.grid(row=5, column=0)

#return_but = tk.Button(root, text="Return to Owner", command=return_to_owner_mode)
#return_but.grid(row=5, column=1)

root.resizable(False, False)
root.mainloop()

