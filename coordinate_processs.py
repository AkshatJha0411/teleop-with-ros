import math
def main(steps_array : list):
    x_y = [0,0]
    direction_1 = 1#y-axis
    direction_2 = 1#positive of any axis
    for _ in steps_array:
        if(_ == "F"):
            if(direction_2 == 1):
                x_y[direction_1] += 1
            if(direction_2 == -1):
                x_y[direction_1] -= 1
        if(_ == "B"):
            if(direction_2 == 1):
                x_y[direction_1] -= 1
            if(direction_2 == -1):
                x_y[direction_1] += 1
        if(_ == "R"):
            if(direction_1 ==1):
                if(direction_2 == 1):
                    direction_2 = 1
                if(direction_2 == -1):
                    direction_2 = -1 
                direction_1 = 0
            else:
                if(direction_2 == 1):
                    direction_2 = -1
                if(direction_2 == -1):
                    direction_2 = 1               
                direction_1 = 1
        if(_ == "L"):
            if(direction_1 ==1):
                if(direction_2 == 1):
                    direction_2 = -1
                if(direction_2 == -1):
                    direction_2 = 1
                direction_1 = 0
            else:
                if(direction_2 == 1):
                    direction_2 = 1
                if(direction_2 == -1):
                    direction_2 = -1
                direction_1 = 1
    return x_y
def return_main(final_coordinates:list):
    return_array = []
    x_cor = final_coordinates[0]
    y_cor = final_coordinates[1]
    if(y_cor == 0 and x_cor > 0):
        for _ in range(x_cor):
            return_array.append("B")
    if(y_cor > 0 and x_cor > 0 ):
        for _ in range(y_cor):
            return_array.append("B")
        return_array.append("R")
        for __ in range(x_cor):
            return_array.append("B")
    if(x_cor == 0 and y_cor > 0):
        for _ in range(y_cor):
            return_array.append("B")






if(__name__ == '__main__'):
    print(main(['B', 'R', 'F', 'L','B']))
            
        
