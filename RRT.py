
import csv
import random
import math


class Point:
    """ Class to define represtation of a point in 2D space"""
    def __init__(self, x, y):
        # x-coordinate of the point
        self.x = x  
        # y-coordinate of the point
        self.y = y 

    def __repr__(self):
        # String represantation of the point
        return f"({self.x}, {self.y})" 

class Node:
    """Definition of class to represent a node in the RRT graph"""
    def __init__(self, id, coordinates, parent=None):
        # Unique identifier for the node
        self.id = id  
        # Coordinates of the node (instance of point)
        self.coordinates = coordinates  
        # Parent node in the graph
        self.parent = parent 

    def __eq__(self, other):
        """Check if two node are equal"""
        return isinstance(other, Node) and self.id == other.id 

    def __repr__(self):
        """String representation of the node"""
        return f"Node(id={self.id}, coordinates={self.coordinates}, parent={self.parent})" 

class Obstacle:
    """Define a class to represent an obstacle in the environment"""
    def __init__(self, center, radius):
        # Center coordinates of the obstacle (instance of Point)
        self.center = center  
        # Radius of the obstacle
        self.radius = radius  


def read_obstacles(filename):
    """ Function to read obstacles from a CSV file
        This can be modified to read values from any file"""
    obstacles = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Ignore comments in the CSV file
            if row and not row[0].startswith('#'): 
                # Parse center coordinates
                center = Point(float(row[0]), float(row[1]))  # Parse center coordinates
                # Parse radius of obstacle
                radius = float(row[2])  
                # Create obstacle object and add to the list
                obstacles.append(Obstacle(center, radius)) 
                # Return list of obstacles
    return obstacles 



def generate_random_point():
    """Function to generate a random point in the 2D space"""
    # Generate random coordinates within a predefined range
    return Point(random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5))  


def distance(p1, p2):
    """ Function to calculate the Euclidian distance between two points"""
    # Calculate distance using the distance formula
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2) 


def is_collision(p1, p2, obstacles):
    """Function to check for collision between two points and a list of obstacles"""
    for obstacle in obstacles:
        if distance(obstacle.center, p1) < obstacle.radius or distance(obstacle.center, p2) < obstacle.radius:
            # Collision detected if the distance to any obstacle's center is less than its radius
            return True  
        # No collision detected
    return False  


def new_node(q_near, q_rand, step_size):
    """Function to generate a new point between two points based on a step"""
    dist = distance(q_near.coordinates, q_rand)
    if dist <= step_size:
        # If the distance is less than or equal to the step size, return the random point
        return q_rand 
    else:
        x = q_near.coordinates.x + (step_size / dist) * (q_rand.x - q_near.coordinates.x)
        y = q_near.coordinates.y + (step_size / dist) * (q_rand.y - q_near.coordinates.y)
        # Generate a new point along the line connecting q_near and q_rand
        return Point(x, y) 


def nearest_node(nodes, target_point):
    """Function to find the nearest node to a target point from a list of nodes"""
    nearest = min(nodes, key=lambda node: distance(node.coordinates, target_point))
    return nearest


def main():
    """Main function to run the RRT algorithm"""
    # Read obstacles from CSV file
    obstacles = read_obstacles(r"C:\Users\alexa\OneDrive\Υπολογιστής\Modern Robotics\Course 4 week 2\results/edges.csv")
    # Maximum number of iterations for the RRT algorithm
    max_iterations = 1000 
    # Maximum number of nodes nodes in the RRT graph
    max_nodes = 200 
    # Step size for generating new nodes
    step_size = 0.1  
    # Start node with ID1 and coordinates (-0.5, -0.5)
    start = Node(1, Point(-0.5, -0.5))
    # Goal node with ID-1 coordinates (0.5, 0.5)
    goal = Node(-1, Point(0.5, 0.5))
    # List to store nodes in the RRT graph
    nodes = [start] 
    # List to store edges between nodes in the RRT graph
    edges = []  

    # RRT algorithm loop
    for _ in range(max_iterations):
        if len(nodes) >= max_nodes:
            break  # Stop if maximum number of nodes is reached
        #Generate a random point
        random_point = generate_random_point() 
        # Find the nearest node to the random point
        nearest = nearest_node(nodes, random_point)
        # Generate a new point between nearest and random points
        new_point = new_node(nearest, random_point, step_size)  
        if not is_collision(nearest.coordinates, new_point, obstacles):
            # Create a new node with parent as nearest
            new_node_obj = Node(len(nodes) + 1, new_point, nearest) 
            # Add the new node to the list of nodes
            nodes.append(new_node_obj) 
            # Add the edge between nearest and new node
            edges.append((nearest, new_node_obj)) 
            if distance(new_point, goal.coordinates) <= step_size:
                break  # Stop if the new point is within the step size of the goal

    # Reconstruct the path from goal to start
    path = []
    current_node = nodes[-1]
    while current_node != start:
        path.append(current_node)
        current_node = current_node.parent
    path.append(start)
    path.reverse()  # Reverse the path to get it from start to goal

    # Write nodes, edges, and path to CSV files
    with open(r"C:\Users\alexa\OneDrive\Υπολογιστής\Modern Robotics\Course 4 week 2\results/nodes.csv", 'w') as nodes_file:
        writer = csv.writer(nodes_file)
        for node in nodes:
            writer.writerow([node.id, node.coordinates.x, node.coordinates.y])

    with open(r"C:\Users\alexa\OneDrive\Υπολογιστής\Modern Robotics\Course 4 week 2\results/edges.csv", 'w') as edges_file:
        writer = csv.writer(edges_file)
        writer.writerow(["# node1 num", "node2 num", "cost"])
        for edge in edges:
            writer.writerow([edge[0].id, edge[1].id, distance(edge[0].coordinates, edge[1].coordinates)])

    with open(r"C:\Users\alexa\OneDrive\Υπολογιστής\Modern Robotics\Course 4 week 2\results/path.csv", 'w') as path_file:
        writer = csv.writer(path_file)
        writer.writerow([node.id for node in path])

# Execute the main function if the script is run directly
if __name__ == "__main__":
    main()