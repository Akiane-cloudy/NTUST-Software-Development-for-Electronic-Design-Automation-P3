#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <regex>
#include <queue>
#include <limits>
using namespace std;

struct Point {
    int x, y;
    Point(int _x = -1, int _y = -1) : x(_x), y(_y) {}
};

struct Net {
    int id;
    Point source, destination;
    int halfPerimeterWireLength;
    vector<Point> path;
    
    Net(int _id, Point src, Point dst)
        : id(_id), source(src), destination(dst),
          halfPerimeterWireLength(abs(src.x - dst.x) + abs(src.y - dst.y)) {}
};

enum class Direction { NONE, UP, DOWN, LEFT, RIGHT };

struct Node {
    Point position;
    double gScore, fScore;
    Direction direction;
    Node* parent;
    
    Node(int x, int y, double g, double f, Direction dir, Node* p)
        : position(x, y), gScore(g), fScore(f), direction(dir), parent(p) {}
};

struct NodeComparator {
    bool operator()(Node* a, Node* b) const {
        return a->fScore > b->fScore;
    }
};

struct NetComparator {
    bool operator()(const Net& a, const Net& b) const {
        return a.halfPerimeterWireLength > b.halfPerimeterWireLength;
    }
};

inline Direction indexToDirection(int idx) {
    switch (idx) {
        case 0: return Direction::UP;
        case 1: return Direction::DOWN;
        case 2: return Direction::LEFT;
        case 3: return Direction::RIGHT;
        default: return Direction::NONE;
    }
}

// Grid dimensions
int rows, columns;

// Cost parameters  
double propagationLoss, crossingLoss, bendingLoss;

// Routing resources
vector<vector<int>> cellUsage;        // Track usage of each grid cell
vector<vector<double>> gScores;       // g-scores for A* algorithm
vector<vector<bool>> visitedCells;    // Closed set for A* algorithm
vector<Net> nets;                     // All nets to be routed

// Direction vectors: Up, Down, Left, Right
const int dy[4] = { 1, -1,  0,  0};
const int dx[4] = { 0,  0, -1,  1};

inline double heuristic(const Point &a, const Point &b) {
    return (abs(a.x - b.x) + abs(a.y - b.y)) * propagationLoss;
}

// Initialize data from input file
void initialize(const string& inputFile) {
    ifstream inputStream(inputFile);
    if (!inputStream) {
        cerr << "Error: Cannot open input file " << inputFile << endl;
        exit(1);
    }
    
    regex gridPattern(R"(grid\s+(\d+)\s+(\d+))");
    regex propagationPattern(R"(propagation\s+loss\s+([\d.]+))");
    regex crossingPattern(R"(crossing\s+loss\s+([\d.]+))");
    regex bendingPattern(R"(bending\s+loss\s+([\d.]+))");
    regex netCountPattern(R"(num\s+net\s+(\d+))");

    string line;
    int netCount = 0;
    
    while (getline(inputStream, line)) {
        smatch matches;
        
        if (regex_search(line, matches, gridPattern)) {
            columns = stoi(matches[1]);
            rows = stoi(matches[2]);
            
            cellUsage.assign(rows, vector<int>(columns, 0));
            gScores.assign(rows, vector<double>(columns, numeric_limits<double>::max()));
            visitedCells.assign(rows, vector<bool>(columns, false));
        }
        else if (regex_search(line, matches, propagationPattern)) {
            propagationLoss = stod(matches[1]);
        }
        else if (regex_search(line, matches, crossingPattern)) {
            crossingLoss = stod(matches[1]);
        }
        else if (regex_search(line, matches, bendingPattern)) {
            bendingLoss = stod(matches[1]);
        }
        else if (regex_search(line, matches, netCountPattern)) {
            netCount = stoi(matches[1]);
            
            for (int i = 0; i < netCount; i++) {
                int id, x1, y1, x2, y2;
                inputStream >> id >> x1 >> y1 >> x2 >> y2;
                nets.emplace_back(id, Point(x1, y1), Point(x2, y2));
                cellUsage[y1][x1]++; // Increment usage for source
                cellUsage[y2][x2]++; // Increment usage for destination
            }
        }
    }
}

// A* pathfinding algorithm for each net
void aStarRouting(Net &net) {
    // Reset data structures for this routing
    for (int y = 0; y < rows; ++y) {
        fill(gScores[y].begin(), gScores[y].end(), numeric_limits<double>::max());
        fill(visitedCells[y].begin(), visitedCells[y].end(), false);
    }

    // Priority queue for open set
    priority_queue<Node*, vector<Node*>, NodeComparator> openSet;
    
    // Create start node
    Node* startNode = new Node(
        net.source.x, net.source.y,
        0.0,
        heuristic(net.source, net.destination),
        Direction::NONE, nullptr
    );
    
    gScores[net.source.y][net.source.x] = 0.0;
    openSet.push(startNode);

    Node* goalNode = nullptr;
    
    // A* main loop
    while (!openSet.empty()) {
        Node* currentNode = openSet.top();
        openSet.pop();
        
        int x = currentNode->position.x;
        int y = currentNode->position.y;
        
        // Skip if already visited
        if (visitedCells[y][x]) { 
            delete currentNode;
            continue; 
        }
        
        visitedCells[y][x] = true;
        
        // Check if destination reached
        if (x == net.destination.x && y == net.destination.y) {
            goalNode = currentNode;
            break;
        }

        // Explore neighbors in all four directions
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            // Skip invalid positions
            if (newX < 0 || newX >= columns || newY < 0 || newY >= rows) continue;
            if (visitedCells[newY][newX]) continue;

            // Calculate cost components
            int crossingCount = max(0, cellUsage[newY][newX]);
            Direction newDirection = indexToDirection(i);
            int bendingCount = (currentNode->direction == Direction::NONE || 
                                currentNode->direction == newDirection) ? 0 : 1;
            
            // Total cost calculation
            double waveguideLoss =  propagationLoss + 
                                    crossingLoss * crossingCount + 
                                    bendingLoss * bendingCount;
            double tentativeGScore = currentNode->gScore + waveguideLoss;
            
            // Check if this path is better
            if (tentativeGScore < gScores[newY][newX]) {
                gScores[newY][newX] = tentativeGScore;
                double estimatedTotal = tentativeGScore + heuristic(Point(newX, newY), net.destination);
                
                openSet.push(new Node(
                    newX, newY, 
                    tentativeGScore, 
                    estimatedTotal,
                    newDirection, 
                    currentNode
                ));
            }
        }
    }

    // Check if path was found
    if (!goalNode) {
        cerr << "Warning: No path found for Net " << net.id << endl;
        return;
    }

    // Reconstruct and store path
    vector<Point> reversePath;
    for (Node* p = goalNode; p; p = p->parent) {
        reversePath.push_back(p->position);
    }
    net.path.assign(reversePath.rbegin(), reversePath.rend());

    // Update usage counts for cells in the path
    for (const auto &point : net.path) {
        cellUsage[point.y][point.x]++;
    }

    // Clean up memory
    while (!openSet.empty()) {
        delete openSet.top();
        openSet.pop();
    }
    
    for (Node* p = goalNode; p;) {
        Node* next = p->parent;
        delete p;
        p = next;
    }
}

// Write routing result to output file
void writeOutput(const string& outputFile) {
    ofstream outputStream(outputFile);
    if (!outputStream) {
        cerr << "Error: Cannot open output file " << outputFile << endl;
        exit(1);
    }
    
    for (const auto &net : nets) {
        int segmentCount = net.path.size() - 1;
        outputStream << net.id << " " << segmentCount << endl;
        
        for (int i = 0; i < segmentCount; i++) {
            const auto &p1 = net.path[i];
            const auto &p2 = net.path[i + 1];
            outputStream << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
        }
    }
}

int main(int argc, char* argv[]) {
    // Check command line arguments
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << endl;
        return 1;
    }
    
    // Process input and route nets
    initialize(argv[1]);
    sort(nets.begin(), nets.end(), NetComparator());
    
    for (auto &net : nets) {
        aStarRouting(net);
    }
    
    // Generate output file
    writeOutput(argv[2]);
    return 0;
}
