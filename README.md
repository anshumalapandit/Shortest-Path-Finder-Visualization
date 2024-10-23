
# Shortest Path Finder Visualizer

# Overview
The Shortest Path Finder Visualizer is an interactive Java application that allows users to visualize the process of finding the shortest path between two points on a randomly generated grid. The application implements two popular pathfinding algorithms: Dijkstra's Algorithm and A*. It is designed to help users understand how these algorithms work while providing real-time performance metrics for computational efficiency.

# Features
Random Grid Generation: The grid is generated with free spaces and obstacles, with a 30% chance for any cell to be an obstacle, adding randomness to the visualization.
Interactive GUI: The graphical user interface allows users to easily select the start and end points on the grid using the mouse. Users can also choose between Dijkstra’s and A* algorithms for pathfinding.
Algorithms:
Dijkstra's Algorithm : Finds the shortest path in a weighted graph without any heuristic optimization.
A* Algorithm: A more efficient pathfinding algorithm that uses heuristics to guide the search, typically providing faster results.
Reset and Cancel Options: At any time, users can reset the grid to start fresh or cancel the current pathfinding operation.
Performance Metrics: The time taken by each algorithm to find the shortest path is displayed in nanoseconds, offering insight into the efficiency of the chosen algorithm.
# Technologies Used
Java: The core programming language used for the entire application.
AWT (Abstract Window Toolkit): Used to build the interactive graphical user interface (GUI).
Pathfinding Algorithms: Both Dijkstra and A* algorithms are implemented from scratch to showcase their differences and efficiency.
# License
This project is licensed under the MIT License. See the LICENSE file for details.

# Author
Anshumala
Developed this project from scratch as a learning tool to visualize the differences between Dijkstra's and A* algorithms in real-time. The project showcases proficiency in algorithm development, Java programming, and GUI creation.

