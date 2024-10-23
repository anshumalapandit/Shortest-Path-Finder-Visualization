/*
 * This software is licensed under the MIT License.
 * Copyright (c) 2024 Anshumala
 * See the LICENSE file for more details.
 */
package shortestpath;
/**
 *
 * @author Anshumala
 */
import java.awt.*;
import java.awt.event.*;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class Node implements Comparable<Node> {

    int x, y, distance;
    Node parent;

    Node(int x, int y, int distance) {
        this.x = x;
        this.y = y;
        this.distance = distance;
        this.parent = null;
    }

    @Override
    //The compareTo method is used to compare nodes based on their distances, enabling the priority queue to function correctly.
    public int compareTo(Node other) {
        return Integer.compare(this.distance, other.distance);
    }
}

class Graph {

    private final int width, height;
    private final int[][] grid;
    //grid: Stores the state of each cell (1 for free space, 0 for obstacles).
    private final int[][] distances;
    private final boolean[][] visited;
    private final Node[][] parents;
    private final Random random;

    Graph(int width, int height) {
        this.width = width;
        this.height = height;
        grid = new int[width][height];
        distances = new int[width][height];
        visited = new boolean[width][height];
        parents = new Node[width][height];
        random = new Random();
        reset();
    }

    void reset() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                distances[i][j] = Integer.MAX_VALUE;
                visited[i][j] = false;
                grid[i][j] = (random.nextInt(100) < 30) ? 0 : 1; // 30% chance of obstacle
                parents[i][j] = null;
            }
        }
    }

    List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        int[][] directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (int[] dir : directions) {
            int newX = node.x + dir[0];
            int newY = node.y + dir[1];
            if (isValid(newX, newY) && grid[newX][newY] == 1 && !visited[newX][newY]) {
                neighbors.add(new Node(newX, newY, distances[newX][newY]));
            }
        }
        return neighbors;
    }

    void setStart(int x, int y) {
        distances[x][y] = 0;
        parents[x][y] = null;
    }

    boolean isVisited(int x, int y) {
        return visited[x][y];
    }

    void visit(int x, int y) {
        visited[x][y] = true;
    }

    int getDistance(int x, int y) {
        return distances[x][y];
    }

    void updateDistance(int x, int y, int newDist, Node parent) {
        distances[x][y] = newDist;
        parents[x][y] = parent;
    }

    boolean isValid(int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    // Dijkstra's Algorithm implementation
    //A priority queue is used to explore the nearest node first.
//Each neighbor's distance is updated if a shorter path is found.
    void dijkstra(int startX, int startY, Label timeLabel) {
        long startTime = System.nanoTime();
        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(startX, startY, 0));
        setStart(startX, startY);

        while (!pq.isEmpty()) {
            Node current = pq.poll();

            if (isVisited(current.x, current.y)) {
                continue;
            }
            visit(current.x, current.y);

            for (Node neighbor : getNeighbors(current)) {
                int newDist = getDistance(current.x, current.y) + 1; // Each step costs 1
                if (newDist < getDistance(neighbor.x, neighbor.y)) {
                    updateDistance(neighbor.x, neighbor.y, newDist, current);
                    pq.add(new Node(neighbor.x, neighbor.y, newDist));
                }
            }
        }
        long endTime = System.nanoTime();
        long duration = endTime - startTime;
        timeLabel.setText("D Time: " + duration + "ns");

    }

    // A* Algorithm implementation
    void aStar(int startX, int startY, int endX, int endY, Label timeLabel) {
        long startTime = System.nanoTime();
        PriorityQueue<Node> pq = new PriorityQueue<>();
        pq.add(new Node(startX, startY, 0));
        setStart(startX, startY);

        while (!pq.isEmpty()) {
            Node current = pq.poll();

            if (isVisited(current.x, current.y)) {
                continue;
            }
            visit(current.x, current.y);

            if (current.x == endX && current.y == endY) {
                return; // Path found
            }
            for (Node neighbor : getNeighbors(current)) {
                int newDist = getDistance(current.x, current.y) + 1; // Each step costs 1
                if (newDist < getDistance(neighbor.x, neighbor.y)) {
                    updateDistance(neighbor.x, neighbor.y, newDist, current);
                    pq.add(new Node(neighbor.x, neighbor.y, newDist));
                }
            }
        }
        long endTime = System.nanoTime();
        long duration = endTime - startTime;
        timeLabel.setText("A* Time: " + duration + "ns");
    }

    void drawPath(Graphics g, int cellSize, int endX, int endY) {
        //drawPath: Fills in the path from the end node back to the start using the parent nodes.
        Node current = parents[endX][endY];
        g.setColor(Color.BLUE);
        while (current != null) {
            g.fillRect(current.x * cellSize, current.y * cellSize, cellSize, cellSize);
            current = parents[current.x][current.y]; // Move to parent
        }
    }

    void drawObstacles(Graphics g, int cellSize) {
        //drawObstacles: Fills in the cells marked as obstacles in black.
        g.setColor(Color.BLACK);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (grid[i][j] == 0) { // Obstacle
                    g.fillRect(i * cellSize, j * cellSize, cellSize, cellSize);
                }
            }
        }
    }
}

class ShortestPath extends Frame implements ActionListener, MouseListener {

    private final Graph graph;
    private final int cellSize = 25;
    private final int gridWidth = 20;
    private final int gridHeight = 20;
    private int startX = -1, startY = -1;
    private int endX = -1, endY = -1;
    private boolean isStartSet = false, isEndSet = false;
    private Button setStartButton, setEndButton, resetButton, findPathButton, cancelButton;
    private Choice algorithmChoice;
    private boolean selectingStart = false, selectingEnd = false;
    private Panel gridPanel;
    private final Label timeLabel;

    ShortestPath() {
        graph = new Graph(gridWidth, gridHeight);
        setTitle("Path Finder Visualizer");
        setSize(gridWidth * cellSize + 250, gridHeight * cellSize + 50);
        setLayout(new BorderLayout());

        Panel sidePanel = new Panel();
        sidePanel.setLayout(new GridBagLayout());
        sidePanel.setPreferredSize(new Dimension(200, gridHeight * cellSize));
        add(sidePanel, BorderLayout.WEST);

        GridBagConstraints gbc = new GridBagConstraints();
        gbc.fill = GridBagConstraints.HORIZONTAL;
        gbc.insets = new Insets(15, 10, 15, 10);

        setStartButton = new Button("Set Start Point");
        setEndButton = new Button("Set End Point");
        resetButton = new Button("Reset");
        findPathButton = new Button("Find Path");
        cancelButton = new Button("Cancel");  // Cancel button added
        algorithmChoice = new Choice();
        algorithmChoice.add("Dijkstra");
        algorithmChoice.add("A*");
       

        Label algorithmLabel = new Label("Select Algorithm:");

        gbc.gridx = 0;
        gbc.gridy = 0;
        sidePanel.add(setStartButton, gbc);

        gbc.gridy++;
        sidePanel.add(setEndButton, gbc);

        gbc.gridy++;
        sidePanel.add(algorithmLabel, gbc);

        gbc.gridy++;
        sidePanel.add(algorithmChoice, gbc);

        gbc.gridy++;
        sidePanel.add(findPathButton, gbc);

        gbc.gridy++;
        sidePanel.add(resetButton, gbc);

        gbc.gridy++;
        sidePanel.add(cancelButton, gbc); // Adding Cancel button to the layout
        timeLabel = new Label("Time: ");
        gbc.gridy++;
        sidePanel.add(timeLabel, gbc);
        
        gridPanel = new Panel() {
            @Override
            public void paint(Graphics g) {
                super.paint(g);
                for (int i = 0; i < gridWidth; i++) {
                    for (int j = 0; j < gridHeight; j++) {
                        g.drawRect(i * cellSize, j * cellSize, cellSize, cellSize);
                    }
                }
                graph.drawObstacles(g, cellSize);

                if (isStartSet) {
                    g.setColor(Color.GREEN);
                    g.fillRect(startX * cellSize, startY * cellSize, cellSize, cellSize);
                }

                if (isEndSet) {
                    g.setColor(Color.RED);
                    g.fillRect(endX * cellSize, endY * cellSize, cellSize, cellSize);
                }

                // Draw the path after running the algorithm
                if (isStartSet && isEndSet && graph.isVisited(endX, endY)) {
                    graph.drawPath(g, cellSize, endX, endY);
                }
            }
        };
    
        gridPanel.addMouseListener(this);
        // this- for current object i.e shortest path
        add(gridPanel, BorderLayout.CENTER);
          // listen and respond
        setStartButton.addActionListener(this);
        setEndButton.addActionListener(this);
        resetButton.addActionListener(this);
        findPathButton.addActionListener(this);
        cancelButton.addActionListener(this);  // Add ActionListener for Cancel button

        // Add WindowAdapter to handle window closing
        // window - our frame main interface
        addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent we) {
                System.exit(0); // Close the application when the window is closed
            }
        });

        setVisible(true);
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        int x = e.getX() / cellSize;
        int y = e.getY() / cellSize;

        if (selectingStart) {
            if (graph.isValid(x, y)) {
                startX = x;
                startY = y;
                isStartSet = true;
                selectingStart = false;
                graph.setStart(startX, startY);
            }
        } else if (selectingEnd) {
            if (graph.isValid(x, y)) {
                endX = x;
                endY = y;
                isEndSet = true;
                selectingEnd = false;
            }
        }

        gridPanel.repaint();
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (e.getSource() == setStartButton) {
            selectingStart = true;
            selectingEnd = false;
        } else if (e.getSource() == setEndButton) {
            selectingEnd = true;
            selectingStart = false;
        } else if (e.getSource() == resetButton) {
            graph.reset();
            startX = -1;
            startY = -1;
            endX = -1;
            endY = -1;
            isStartSet = false;
            isEndSet = false;
            timeLabel.setText("Time: ");
            gridPanel.repaint();
        } else if (e.getSource() == findPathButton) {
            if (isStartSet && isEndSet) {
                if (algorithmChoice.getSelectedItem().equals("Dijkstra")) {
                    graph.dijkstra(startX, startY, timeLabel);
                } else {
                    graph.aStar(startX, startY, endX, endY, timeLabel);
                }
                gridPanel.repaint();
            }
        } else if (e.getSource() == cancelButton) { // Cancel button functionality
            // Reset any selections and stop any pathfinding operations
            selectingStart = false;
            selectingEnd = false;
            graph.reset();
            startX = -1;
            startY = -1;
            endX = -1;
            endY = -1;
            isStartSet = false;
            isEndSet = false;
            gridPanel.repaint(); // Repaint to reflect cancellation
        }
    }

    @Override
    public void mousePressed(MouseEvent e) {
    }

    @Override
    public void mouseReleased(MouseEvent e) {
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    public static void main(String[] args) {
        ShortestPath sp= new ShortestPath();
        
    }
}
