#include <iostream>
#include <unordered_set>
#include <vector>
#include <stack>
#include <unordered_map>
#include <queue>
#include <functional>

class graph{
    public:
    std::unordered_map<int, std::unordered_map<int,int>> graph;

    bool DFS(int src, int dest, std::unordered_set <int> & visited){
        if (src==dest){
            return true;
        }
        visited.insert(src);
        for (const auto& neighbours: graph[src]){
            int adjnode = neighbours.first;
            if(!visited.count(adjnode)){
                if (DFS(adjnode, dest, visited)) {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<int> DFS_stack(int src, int dest){
        std::stack<std::vector<int>> todo; // stack todo to store all of the possible paths
        std::unordered_set<int> visited; // set visited to store visited nodes

        todo.push({src}); // add the source to the stack

        while(!todo.empty()){ // as long as the stack is not empty
            // Step 1: assign the current path to be the more recently pushed path (LIFO structure of stack)
            std::vector<int> currentPath = todo.top(); 
            todo.pop();
            // Step 2: get the last node in the path, and check if we have found a valid path -> return that path
            int currentNode = currentPath.back();
            if(currentNode == dest){
                return currentPath;
            }
            // Step 3: at this point, the path has not reached the destination, so we want to get all of the next
            // paths from the last node in the path (called currennt Node)
            // check if this node has been visited (cycle detection), then insert it into the visited set and 
            // add all of its adj nodes to the current paths and add it to the stack
            if (!visited.count(currentNode)){
                visited.insert(currentNode); // add to visited set

                // for each adj node to the current last node, append it to the current path and push each
                // path to the stack
                for(const auto & neighbours : graph[currentNode]){
                    int adjNode = neighbours.first;
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(adjNode);
                    todo.push(newPath);
                }
            }
        }
        // if at this point no path has been found, then no path exists!
        return {};
    }

    std::vector<int> BFS (int src, int dest){
        std::queue<std::vector<int>> todo; // queue todo to store all of the possible paths
        std::unordered_set<int> visited; // visited set

        todo.push({src}); // add the source vertex to the queue, to be dequeued first

        // While loop that runs as long as the queue is not empty
        while(!todo.empty()){
            // Step 1: let the current path be the least recently pushed path (FIFO)
            std::vector<int> currentPath = todo.front();
            todo.pop();
            // Step 2: grab the current node which represents the last node in the path and return
            // the path if it is the destination!
            int currentNode = currentPath.back();
            if(currentNode == dest){
                return currentPath;
            }
            // Step 3: path is not found, so visit the current Node and append all of its adjacent nodes
            // to the current path and push to the queue
            if (!visited.count(currentNode)){
                visited.insert(currentNode);
                // For each adjacent node to the current node, add to the current path and push to queue
                for (const auto& neighbours: graph[currentNode]){
                    int adjNode = neighbours.first;
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(adjNode);
                    todo.push(newPath);
                }
            }
            
        }
        // no paths found, return!
        return {};
    }


    std::vector <int> Dijkistra(int src, int dest){

        // priority queue set up by chat gpt:
        // the pq stores a pair of {total edge weight, path}
        using P = std::pair<int, std::vector<int>>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> todo;

        std::unordered_set <int> visited; // visited set to avoid cycles

        todo.push({0, {src}}); //enqueue source with a total edge weight of 0

        // as long as the queue is not empty:
        while (!todo.empty()){
            // Step 1: get the shortest current path by removing minimum from the PQ
            std::pair <int, std::vector<int>> currentPath = todo.top();
            todo.pop();

            // Step 2: get the current node (or the last node in the path) and return if destination is reached
            int currentNode = currentPath.second.back();
            if(currentNode == dest){
                return currentPath.second;
            }

            // Step 3: path not found- append all of the adjacent nodes to the current path and
            // add all of them to the queue
            if (!visited.count(currentNode)){
                visited.insert(currentNode);
                // for each adjacent node to the current node, add a pair of {weight, path} to the PQ
                // by appending the node to the current path, and adding the weights
                for (const auto & neighbours: graph[currentNode]){
                    int adjNode = neighbours.first;
                    int weight = neighbours.second;
                    std::vector<int> newPath = currentPath.second;
                    newPath.push_back(adjNode);
                    todo.push({currentPath.first + weight, newPath});
                }
            }
        }
        // no path found
        return {};
    }
};

int main() {
    graph g;

    // Build the graph using the image structure
    g.graph[1][2] = 1; // A -> B
    g.graph[1][4] = 1; // A -> D
    g.graph[2][3] = 1; // B -> C
    g.graph[3][7] = 1; // C -> G
    g.graph[4][2] = 1; // D -> B
    g.graph[4][5] = 1; // D -> E
    g.graph[4][6] = 1; // D -> F
    g.graph[5][7] = 1; // E -> G
    g.graph[6][4] = 1; // F -> D

    int src = 1;  // A
    int dest = 3; // C

    // DFS (recursive, just checks if a path exists)
    std::unordered_set<int> visited;
    bool pathExists = g.DFS(src, dest, visited);
    std::cout << "DFS path exists: " << (pathExists ? "Yes" : "No") << std::endl;

    // DFS with stack (returns path)
    std::vector<int> dfsPath = g.DFS_stack(src, dest);
    std::cout << "DFS_stack path: ";
    for (int node : dfsPath) std::cout << node << " ";
    std::cout << std::endl;

    // BFS (returns shortest unweighted path)
    std::vector<int> bfsPath = g.BFS(src, dest);
    std::cout << "BFS path: ";
    for (int node : bfsPath) std::cout << node << " ";
    std::cout << std::endl;


    graph g2;

    g2.graph[1][2] = 3; // A → B
    g2.graph[2][3] = 3; // B → C
    g2.graph[3][4] = 6; // C → D
    g2.graph[4][5] = 3; // D → E
    g2.graph[1][4] = 5; // A → D
    g2.graph[1][6] = 4; // A → F
    g2.graph[6][5] = 5; // F → E
    g2.graph[4][1] = 1; // D → A
    g2.graph[6][4] = 2; // F → D

     src = 1;   // A
     dest = 5;  // E

    // Dijkstra (returns shortest weighted path)
    std::vector<int> dijkstraPath = g.Dijkistra(src, dest);
    std::cout << "Dijkstra path: ";
    for (int node : dijkstraPath) std::cout << node << " ";
    std::cout << std::endl;

    return 0;
}