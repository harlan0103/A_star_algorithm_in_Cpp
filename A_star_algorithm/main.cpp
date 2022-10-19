/////////////////////////////////////////////////////////////
// Implementation of A* algorithm with command line display
// Author: Haoran Liang
/////////////////////////////////////////////////////////////

#include <iostream>
#include <vector>

using namespace std;

// Display board
const vector<vector<int>> board = {

    {0,0,0,0,0,0,2},
    {0,-1,-1,-1,-1,-1,0},
    {0,0,0,0,0,0,-1},
    {0,-1,0,0,0,-1,-1},
    {-1,-1,0,-1,0,0,0},
    {0,0,0,0,0,-1,-1},
    {1,0,0,0,0,0,0}
};

const int board_height = (int)board.size();
const int board_width = board_height == 0 ? 0 : (int)board[0].size();

namespace AStarAlgorithm {
    // Node class contains position of the node and its distance from start to end
    class Node {
    public:

        // Constructor to initialize a node with position and distance
        Node(pair<int, int> _position = { 0, 0 }, int _distance = 0) {
            position = _position;
            distance = _distance;
            idx = position.first * board_width + position.second;
        }

        // Default destructor
        ~Node() {}

        // Return distance from start to this node to end
        inline int getDistance() const { return distance; }

        // Set a new distance
        inline void setDistance(int& _dis) { distance = _dis; }

        // Return current (x,y) coordinate in pair
        inline pair<int, int> getPosition() { return position; }

        // Return current (x,y) coordinate in index format
        inline int getIdx() const { return idx; }

        // Compare two nodes with distance value
        bool operator< (const Node& n) {
            if (distance <= n.getDistance()) {
                return true;
            }
            else {
                return false;
            }
        }

        // Compare two nodes with distance value
        bool operator> (const Node& n) {
            if (distance > n.getDistance()) {
                return true;
            }
            else {
                return false;
            }
        }
    private:

        // (x,y) coordinate in pair
        pair<int, int> position;

        // (x,y) coordinate in index format
        int idx;

        // distance from start to this node to end point
        int distance;
    };

    // Override outstream for node objects
    ostream& operator<< (ostream& out, Node& n) {
        out << "(" << n.getPosition().first << "," << n.getPosition().second << ") -> " << n.getDistance();
        return out;
    }

    // Heap class to store node objects in minimum distance order
    // Always keep the smallest distance node as the top
    class MyHeap {
    public:
        // Constructor to initialize the size of vecotr to 255 with zero count
        MyHeap() {
            list.resize(255);
            count = 0;
        }

        ~MyHeap() {}

        // Insert a new node object into the list
        void insert(Node& node) {
            if (count + 1 > list.capacity()) {
                return;
            }
            list[++count] = node;
            shiftUp(count);
        }

        // Return and remove the top node object from the list 
        Node pop() {
            Node n = list[1];
            std::swap(list[1], list[count]);
            count--;
            shiftDown();

            return n;
        }

        // Return current size of heap
        inline int size() { return count; }

        // Return if current heap is empty
        inline bool empty() { return count == 0; }

        // Debug helper function to print out current nodes in heap
        void print() {
            cout << "[ ";
            for (int i = 1; i <= count - 1; i++) {
                cout << list[i] << ", ";
            }

            cout << list[count] << " ]" << endl;
        }

    private:

        // Use vector as data structure to hold nodes
        vector<Node> list;

        // Current number of nodes in heap
        int count;

        // Shift last node up
        void shiftUp(int position) {
            while (position > 1 && list[position / 2] > list[position]) {
                std::swap(list[position / 2], list[position]);
                position /= 2;
            }
        }

        // Shift first node down
        void shiftDown() {
            int i = 1;
            while (i <= count) {
                int l = i * 2 <= count ? i * 2 : i;
                int r = i * 2 + 1 <= count ? i * 2 + 1 : i;

                int min = list[l] < list[r] ? l : r;
                if (list[i] > list[min]) {
                    std::swap(list[i], list[min]);
                    i = min;
                }
                else {
                    break;
                }
            }
        }
    };

}

// Display board on command line with different entry pattern
void drawBoard(const vector<vector<int>>& board) {
    int height = board_height;
    int width = board_width;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << "*-----";
        }
        cout << "*" << endl;

        for (int idx = 0; idx < 2; idx++) {
            for (int k = 0; k < width; k++) {
                int entry = board[i][k];
                if (entry == 0) {
                    cout << "|     ";
                }
                else if (entry == 1) {
                    cout << "|OOOOO";
                }
                else if (entry == 2) {
                    cout << "|*****";
                }
                else if (entry == -1) {
                    cout << "|XXXXX";
                }
                else if (entry == 3) {
                    cout << "|+++++";
                }
            }
            cout << "|" << endl;
        }
    }
    // Draw end bound
    for (int j = 0; j < width; j++) {
        cout << "*-----";
    }
    cout << "*" << endl;
    cout << endl;
}

// Get neighbors of input node
vector<pair<int, int>> getNeighbors(AStarAlgorithm::Node& currentNode) {
    vector<pair<int, int>> list;

    int x = currentNode.getPosition().first;
    int y = currentNode.getPosition().second;

    if (x - 1 >= 0 && board[x - 1][y] != -1) {
        list.push_back({ x - 1, y });
    }

    if (x + 1 < board_height && board[x + 1][y] != -1) {
        list.push_back({ x + 1, y });
    }

    if (y - 1 >= 0 && board[x][y - 1] != -1) {
        list.push_back({ x, y - 1 });
    }

    if (y + 1 < board_width && board[x][y + 1] != -1) {
        list.push_back({ x, y + 1 });
    }

    return list;
}

// Implmentation of A* algorithm
void pathFinding(const int& startPos, const int& endPos, const vector<vector<int>>& board, vector<int>& path) {
    AStarAlgorithm::MyHeap myHeap;
    vector<int> costSoFar(board_height * board_width, 0);
    vector<int> cameFrom(board_height * board_width, -1);

    int start_x = startPos / board_width;
    int start_y = startPos % board_width;

    int end_x = endPos / board_width;
    int end_y = endPos % board_width;

    cout << "start position: (" << start_x << ", " << start_y << ")" << endl;
    cout << "end position : (" << end_x << ", " << end_y << ")" << endl << endl;

    // Add start node into the priority queue
    AStarAlgorithm::Node startNode({ start_x, start_y }, 0);
    myHeap.insert(startNode);

    cout << "=== Start path finding ===" << endl << endl;
    // Start path finding
    while (!myHeap.empty()) {
        // Pop out the current top node
        AStarAlgorithm::Node top = myHeap.pop();
        int top_x = top.getPosition().first;
        int top_y = top.getPosition().second;

        if (top_x == end_x && top_y == end_y) {
            break;
        }

        // Find all neighbors from four directions
        vector<pair<int, int>> neighbors = getNeighbors(top);
        for (pair<int, int> neighbor : neighbors) {
            int neighbor_x = neighbor.first;
            int neighbor_y = neighbor.second;

            // Calculate the cost from current top node to this neighbor node
            int newCost = costSoFar[top_x * board_width + top_y] + 1;

            // Check if this neightbor is visited or the new cost is lower than before
            if (costSoFar[neighbor_x * board_width + neighbor_y] == 0 || newCost < costSoFar[neighbor_x * board_width + neighbor_y]) {
                costSoFar[neighbor_x * board_width + neighbor_y] = newCost;

                // Calculate heuristic distance from neighbor node to end node using manhattan distance
                int heuristic = std::abs(end_x - neighbor_x) + std::abs(end_y - neighbor_y);
                int distance = newCost + heuristic;
                AStarAlgorithm::Node neighborNode(neighbor, distance);

                myHeap.insert(neighborNode);
                // Update camefrom list
                cameFrom[neighbor_x * board_width + neighbor_y] = top_x * board_width + top_y;
            }
        }
    }

    // Check if we have valid path
    if (cameFrom[end_x * board_width + end_y] == -1) {
        return;
    }
    else {
        int idx = end_x * board_width + end_y;
        while (idx != start_x * board_width + start_y && idx >= 0) {
            //cout << "idx: " << idx / board_width << "," << idx % board_width << endl;
            path.push_back(idx);
            idx = cameFrom[idx];
        }
    }
}

// Helper function to print legend info for the board
void printHelperMessage() {
    cout << "1) Start point: \t \t \t" << "*-----*" << endl;
    cout << "                \t \t \t" << "|OOOOO|" << endl;
    cout << "                \t \t \t" << "|OOOOO|" << endl;
    cout << "                \t \t \t" << "*-----*" << endl << endl;

    cout << "2) End point:   \t \t \t" << "*-----*" << endl;
    cout << "                \t \t \t" << "|*****|" << endl;
    cout << "                \t \t \t" << "|*****|" << endl;
    cout << "                \t \t \t" << "*-----*" << endl << endl;

    cout << "3) Blocks:      \t \t \t" << "*-----*" << endl;
    cout << "                \t \t \t" << "|XXXXX|" << endl;
    cout << "                \t \t \t" << "|XXXXX|" << endl;
    cout << "                \t \t \t" << "*-----*" << endl << endl;

    cout << "4) Empty path:  \t \t \t" << "*-----*" << endl;
    cout << "                \t \t \t" << "|     |" << endl;
    cout << "                \t \t \t" << "|     |" << endl;
    cout << "                \t \t \t" << "*-----*" << endl << endl;

    cout << "5) Result path: \t \t \t" << "*-----*" << endl;
    cout << "                \t \t \t" << "|+++++|" << endl;
    cout << "                \t \t \t" << "|+++++|" << endl;
    cout << "                \t \t \t" << "*-----*" << endl << endl;
}

// Main function
int main()
{
    // Check if it is a valid board
    if (board_height == 0 && board_width == 0) {
        cout << "=== ERROR ===: Invalid board!" << endl;
        return 1;
    }

    int startPos = -1;
    int endPos = -1;
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            if (board[i][j] == 1) {
                if (startPos != -1) {
                    cout << "=== ERROR ===: Can't have more than 1 start point!" << endl;
                    return 1;
                }
                else {
                    startPos = i * board_width + j;
                }
            }

            if (board[i][j] == 2) {
                if (endPos != -1) {
                    cout << "=== ERROR ===: Can't have more than 1 end point!" << endl;
                    return 1;
                }
                else {
                    endPos = i * board_width + j;
                }
            }
        }
    }

    // Draw board
    cout << "=== Input path finding board ===" << endl << endl;
    drawBoard(board);

    cout << "=== Board legend ===" << endl;
    printHelperMessage();

    // Start path finding
    vector<int> path;
    pathFinding(startPos, endPos, board, path);

    // Draw board if have valid path
    vector<vector<int>> outputBoard = board;
    if (path.size() != 0) {
        cout << "=== RESULT ===: Found a path!" << endl;
        for (int i = 1; i < path.size(); i++) {
            // Last index in path is the end position
            int x = path[i] / board_width;
            int y = path[i] % board_width;
            outputBoard[x][y] = 3;
        }

        drawBoard(outputBoard);
    }
    else {
        cout << "=== RESULT ===: No valid path found!" << endl;
    }

    return 0;
}