import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import numpy as np
import math
import yaml
# import os
# from std_msgs.msg import Bool

from amr_sanitizer_robot.robot_class import DifferentialRobot

#General Functions

def create_position(x, y):
    '''
    Create a PoseStamped message with the given x and y coordinates
    '''
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose

def calc_distnace_betw_points(point1, point2):
    '''
    Calculate the distance between two points
    '''
    point2_x, point2_y = map(float, point2)
    dx = point2_x - point1.position.x
    dy = point2_y - point1.position.y
    return math.sqrt(dx**2 + dy**2)

def find_corners_of_room(room):
    '''
    Return the corners of the room
    - Input: room
    - Output: corners
    '''
    
    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],
        'hallway': [0.0, 2.2, 0.0, 1.0],
        'study': [-5.0, 5.0, -4.0, 0.0],
        'lounge1': [-7.2, 5.0, -8.0, -4.0],
        'lounge2': [5.0, 7.2, -8.0, -5.2]   
    }

    corners = []
    offset = 0.30
    coordinates = room_coordinates[room]

    if room == 'lounge1' or room == 'lounge2':
                room = 'lounge 1'
                
                corners.extend([(7.2-offset, -8.0+offset),
                                    (-7.2+offset,-8.0+offset),
                                    (-7.2+offset,-4.0-offset),
                                    (5.0-offset,-4.0-offset),
                                    (5.0-offset,-5.2-offset),
                                    (7.2-offset,-5.2-offset),
                                    (7.2-offset,-8.0+offset)]
                                    )
    else:
        
        corners.extend([(coordinates[0]+offset,coordinates[2]+offset), #(x_start, y_start)
                            (coordinates[0]+offset,coordinates[3]-offset),  #(x_start, y_end)
                            (coordinates[1]-offset,coordinates[3]-offset),  #(x_end, y_end)
                            (coordinates[1]-offset,coordinates[2]+offset),  #(x_end, y_start)
                            (coordinates[0]+offset,coordinates[2]+offset)]  #(x_start, y_start)
                            )
    
    return corners

def find_center_of_room(room):
    '''
    Return the center of the room
    - Input: room
    - Output: room_center_x, room_center_y
    '''

    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],
        'hallway': [0.0, 2.2, 0.0, 1.0],
        'study': [-5.0, 5.0, -4.0, 0.0],
        'lounge1': [-7.2, 5.0, -8.0, -4.0],
        'lounge2': [5.0, 7.2, -8.0, -5.2]   
    }

    room_centers = {}
    coordinates = room_coordinates[room]
    x_start, x_end, y_start, y_end = coordinates
    room_center_x = (x_start + x_end) / 2
    room_center_y = (y_start + y_end) / 2
    room_centers[room] = (room_center_x, room_center_y)

    return room_center_x, room_center_y

def identify_room(robot_x, robot_y):
    '''
    Check which room the robot is in based on its coordinates
    Input: robot_x, robot_y, this is the position of the robot
    Output: room, the room the robot is in
    '''
    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],
        'hallway': [0.0, 2.2, 0.0, 1.0],
        'study': [-5.0, 5.0, -4.0, 0.0],
        'lounge1': [-7.2, 5.0, -8.0, -4.0],
        'lounge2': [5.0, 7.2, -8.0, -5.2]   
    }
    for room, coordinates in room_coordinates.items():
        x_start, x_end, y_start, y_end = coordinates
        if room == 'lounge1' or room == 'lounge2':
            lounge = [-7.2, 5.0, -8.0, -4.0]
            x_start, x_end, y_start, y_end = lounge
            
        if x_start <= robot_x <= x_end and y_start <= robot_y <= y_end:
            return room
        
    return None

def is_point_in_room(point, corners): #To check whether point is in the room
    '''
    Check if a point is inside a room
    - Input: point, corners
    - Output: inside, True if the point is inside the room, False otherwise
    '''
    x, y = point
    inside = False
    for i in range(len(corners)):
        j = (i + 1) % len(corners)
        if (corners[i][1] > y) != (corners[j][1] > y) and \
                x < (corners[j][0] - corners[i][0]) * (y - corners[i][1]) / \
                (corners[j][1] - corners[i][1]) + corners[i][0]:
            inside = not inside
    return inside

def remove_closest_points(unsanitized_points):
    filtered_coordinates = []
    filtering_threshold = 0.4
    for i in range(len(unsanitized_points)):
        is_close = False
        for j in range(i+1, len(unsanitized_points)):   # (i+1, len(unsanitized_points))
            distance = math.sqrt((unsanitized_points[i][0] - unsanitized_points[j][0])**2 + (unsanitized_points[i][1] - unsanitized_points[j][1])**2)
            if distance < filtering_threshold:
                is_close = True
                break
        if not is_close:  #if point is not close add the point to the filtered_coordinates.
            filtered_coordinates.append(unsanitized_points[i])
    return filtered_coordinates

#Main function as well it is place where general functions communicate with robot which is class.
def main(args=None):
    rclpy.init(args=args)
    robot = DifferentialRobot()
    count=0
    # Wait for localization to complete before starting the route manager
    while robot.localization_complete == False:
        if count==0:
            robot.get_logger().info("Waiting for localization to complete")
            count=1
        rclpy.spin_once(robot)
    robot.get_logger().info("Localization complete, starting Sanitization")

    while robot.map_data is None:
        rclpy.spin_once(robot)
        
    print("Map dimension shape[0], shape[1] :", robot.map_data.shape[0], robot.map_data.shape[1]);

    # Read the rooms to be sanitized from the yaml file
    with open('src/amr_sanitizer_robot/amr_sanitizer_robot/rooms.yaml', 'r') as file:
        rooms_data = yaml.safe_load(file)
    uncompleted_rooms = rooms_data.split()

    original_rooms = np.copy(uncompleted_rooms)

    print ("List of rooms to be sanitized: ", uncompleted_rooms)
    while robot.robot_pose is None:
        rclpy.spin_once(robot)
        print("Waiting for robot pose...")
    
    robot_x = robot.robot_pose.position.x
    robot_y = robot.robot_pose.position.y

    room = identify_room(robot_x, robot_y)
    current_room = room
    print("Robot spawned in", current_room)

    if room in uncompleted_rooms:
        current_room = room
    else:
        nearest_room = None
        min_distance = float('inf')

        for room in uncompleted_rooms:
            room_center_x, room_center_y = find_center_of_room(room)
            distance = calc_distnace_betw_points((robot.robot_pose), (room_center_x, room_center_y))
            if distance < min_distance:
                min_distance = distance
                nearest_room = room

        if nearest_room is not None:
            print("Going to nearest room:", nearest_room)
            room_center_x, room_center_y = find_center_of_room(nearest_room)  #calc. cordinate of nearest center room
            goal_pose = create_position(room_center_x, room_center_y)  #from coordinate create goal position.
            robot.goToPose(goal_pose)  #send goal position to robot
            while not robot.isNavComplete(): #wait untill the goal is reached
                rclpy.spin_once(robot)

            while robot.status != GoalStatus.STATUS_SUCCEEDED: #while goal is not succeed 
                robot.goToPose(goal_pose)   #send goal position to robot
                while not robot.isNavComplete():
                    rclpy.spin_once(robot)
            
            robot_pose = robot.robot_pose  #assign robot_pose to value of robot_pose in the robot
            robot_x = robot.robot_x   #take robot x-postion from robot
            robot_y = robot.robot_y   #take robot y-postion from robot
    
    current_room = identify_room(robot_x, robot_y)  #identify in which room we are in now, using identify_room
    robot.start_energy = True   #This time turn UV lamp, and start sanitazer
    if current_room is not None:  
        print("Robot is in", current_room)  #print in which room we are in now, on terminal.
    else:
        print("Robot is not in any room")

    room_center_x,room_center_y = find_center_of_room(current_room) #for the current_room compute room center (x,y) coordinate
    corners = find_corners_of_room(current_room) #define the corners for the current room

    goal_pose = create_position(room_center_x, room_center_y) #create goal position for center (x,y)
    robot.goToPose(goal_pose)  #send the robot to the center of the current room
    print("I am going to the center of the room")
    while not robot.isNavComplete():
        rclpy.spin_once(robot)

    robot_pose = robot.robot_pose  #assign robot_pose(in main) with the robot_pose(in robot).
    robot_x = robot_pose.position.x   #take robot-x pos. from robot_pose(in main)
    robot_y = robot_pose.position.y   #take robot-y pos. from robot_pose(in main)
    robot_pose = robot.robot_pose 
       
    all_rooms_sanitized = False 
    completed_rooms = []  #collect sanitzed room.
    #While Loop for Sanitizing the all required rooms
    while all_rooms_sanitized == False: #while house is not sanitized, continoue sanitizing
        room_complete = False 
        
        print("Robot is going to the corners of the room")
        for corner in corners:
            goal_pose = create_position(corner[0], corner[1])
            robot.goToPose(goal_pose)
            while not robot.isNavComplete():                
                rclpy.spin_once(robot)

        iter = 0
        while room_complete == False: #while room_complete is False, w/c means room is not sanitized, then False==False, loop continoues 
            # iter = 0
            robot_pose = robot.robot_pose
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y

            unsanitized_points = []
            nearest_coordinate = None

            i,j=0,0
            #print(" unsanitized_points before filtered: ", unsanitized_points)
            for i in range(robot.cell_sanitized.shape[0]):
                for j in range(robot.cell_sanitized.shape[1]): 
                    if robot.cell_energy[i,j] < 0.001:
                        world_x, world_y = robot.map_to_world(i, j)
                        world_x = round(world_x,3)
                        world_y = round(world_y,3)
                        point = (world_x, world_y)
                        if is_point_in_room(point, corners):
                            unsanitized_points.append(point)

            unsanitized_points = list(set(unsanitized_points))  # Remove repeated points

            # Remove points that are too close to each other
            unsanitized_points = remove_closest_points(unsanitized_points) #assign points that are not close to each other to unsanitized_points
            unsanitized_points.sort(key=lambda coord: math.sqrt((coord[0] - robot_x)**2 + (coord[1] - robot_y)**2))

            print("room_corners is: ", corners)
            print("There are ", len(unsanitized_points),"unsanitized points found: ", unsanitized_points)

            # Use the nearest_coordinate for further processing
            while unsanitized_points: #select coord. of min distance from unsanitized_points and navigate
                rclpy.spin_once(robot)
                nearest_coordinate = min(unsanitized_points, key=lambda coord: math.sqrt((coord[0] - robot_x)**2 + (coord[1] - robot_y)**2))
                map_nearest_coordinate = robot.world_to_map(nearest_coordinate[0], nearest_coordinate[1]) #convert nearst point to map cordinate.
                
                #The following code is import just in case it is by change sanitized while we were move somewhere, we remove it from list of unsanitized
                if robot.cell_energy[map_nearest_coordinate[0], map_nearest_coordinate[1]] >= 0.001: #check whether this nearst point is sanitazed.
                   unsanitized_points.remove(nearest_coordinate) #if so, remove this points from unsanitized_points
                   continue #and find other nearst point again.
                
                goal_pose = create_position(nearest_coordinate[0], nearest_coordinate[1]) #if ponit is not sanitized, create goal_pose to navigate to this point
                robot.goToPose(goal_pose) #finally navigate to the point.
                print("I am goint to ",map_nearest_coordinate[0], map_nearest_coordinate[1], "in map index")
                print("with cell energy", robot.cell_energy[map_nearest_coordinate[0], map_nearest_coordinate[1]], "Cell sanitized state", robot.cell_sanitized[map_nearest_coordinate[0], map_nearest_coordinate[1]])

                while not robot.isNavComplete(): #wait until navigation is finished.
                    rclpy.spin_once(robot)

                # Remove sanitized point from unsanitized_points
                unsanitized_points.remove(nearest_coordinate)

                robot_pose = robot.robot_pose
                robot_x = robot_pose.position.x
                robot_y = robot_pose.position.y                             

            print("I am going to write the cell energy matrix to file: src/myfile.txt ")

            np.savetxt('cell_energy.txt', robot.cell_energy, delimiter=',')   # X is an array
            np.savetxt('cell_sanitized.txt', robot.cell_sanitized, delimiter=',')   # X is an array

            room_complete = True #we set to True, for the initialization the following cond. w/c checks unsanitized_points in the room.
            iter +=1

       #We arrive here b/c current room is sanitized. 
       #Preparing information for the next room to be visited                 
        robot_pose = robot.robot_pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y  
        print(current_room," has been sanitized")         

        current_room_center_x, current_room_center_y = find_center_of_room(current_room)
        completed_rooms.append(current_room)  #Add the current_room to the completed_rooms.
        uncompleted_rooms.remove(current_room) #And remove the current_room from rooms to be sanizated.

        print("Completed rooms: ", completed_rooms)
        print("Remaining rooms: ", uncompleted_rooms)


        if set(completed_rooms) == set(original_rooms): #If all rooms are sanitized.
            all_rooms_sanitized = True #set the all_rooms_sanitized is sanitized
            print("All the rooms that required sanitization.")
            break

        # Compute the center of the nearest room
        nearest_room_center = None

        #compute the displacement, to get which room is nearest to current room
        displacement = {}
        for room_name in uncompleted_rooms:
            center_x, center_y = find_center_of_room(room_name) 
            distance = math.sqrt((current_room_center_x - center_x)**2 + (current_room_center_y - center_y)**2)
            displacement[room_name] = distance

        if displacement: #choose the nearst room from the available rooms
            nearest_room = min(displacement, key=displacement.get)
        
        nearest_room_center =  find_center_of_room(nearest_room)   
        
        if nearest_room_center is not None:
            center_x, center_y = nearest_room_center
            current_room = nearest_room
            corners = find_corners_of_room(nearest_room)
            print("Robot is going to the center of the ",nearest_room)
    
        # Go to the center of the nearest room
        if nearest_room_center is not None:
            goal_pose = create_position(nearest_room_center[0], nearest_room_center[1])
            robot.goToPose(goal_pose)
            print("Going to the center of the nearest room")
            while not robot.isNavComplete():
                rclpy.spin_once(robot)     

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally: # Destroy and shutdown the rclpy
        robot.destroy_node()
        rclpy.shutdown()

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
