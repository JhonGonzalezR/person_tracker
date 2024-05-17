from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    LaunchDescription()

    tracker = Node(
        package="person_tracker",
        executable="detect_person",
        
        

    )
    follow = Node(
        package="person_tracker",
        executable="follow_person",
        
    )


  

    return LaunchDescription([tracker,
                              follow])  