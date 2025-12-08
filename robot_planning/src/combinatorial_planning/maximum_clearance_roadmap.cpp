#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include "combinatorial_planning/planning_utils.h" // For pointInObstacle/Border checks

namespace MaxClearanceRoadmap {

    // ==========================================
    // Internal Helper Structures
    // ==========================================
    
    struct GridNode {
        int x, y;
    };

    struct PixelInfo {
        double dist;       // Distance to nearest obstacle
        int sourceObsId;   // ID of the obstacle that is nearest (-1 if none/init)
        bool isVoronoi;    // Is this pixel on the ridge?
        int roadmapNodeIdx; // Map to the final Roadmap vertex index
    };

    // ==========================================
    // Internal Helper Functions
    // ==========================================

    // Helper to check if a grid cell is valid
    bool isValid(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // ==========================================
    // Main Implementation
    // ==========================================

    std::shared_ptr<Roadmap> maximumClearanceRoadmap(const Map& map, double gridResolution) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        // 1. Determine Grid Dimensions
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        
        // Add padding
        minX -= 1.0; minY -= 1.0; maxX += 1.0; maxY += 1.0;

        int width = std::ceil((maxX - minX) / gridResolution);
        int height = std::ceil((maxY - minY) / gridResolution);

        if (width <= 0 || height <= 0) return roadmap;

        // 2. Initialize Grid
        // grid[x][y] stores distance and source obstacle ID
        std::vector<std::vector<PixelInfo>> grid(width, std::vector<PixelInfo>(height, {std::numeric_limits<double>::max(), -1, false, -1}));
        std::queue<GridNode> q;

        // 3. Rasterize Obstacles & Borders (Sources)
        // We treat Borders as Obstacle ID 0
        // We treat Obstacles as IDs 1, 2, 3...
        
        // Rasterize Borders (approximate as boundary)
        // A better approach for GVD inside a polygon is to treat the polygon boundary as the obstacle.
        // We iterate over the grid and check "pointInPolygon" vs "pointInObstacle".
        
        int obsCount = 0;

        // Brute-force initialization (safe & robust for complex shapes)
        // For every pixel, check if it's inside an obstacle or close to border
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                Vertex worldPos(minX + x * gridResolution, minY + y * gridResolution);
                
                // Check if inside any obstacle
                bool insideObstacle = false;
                int foundObsId = -1;

                // Check actual obstacles
                const auto& obstacles = map.obstacles.get_obstacles();
                for (size_t i = 0; i < obstacles.size(); ++i) {
                    if (PlanningUtils::pointInObstacle(worldPos, obstacles[i])) {
                        insideObstacle = true;
                        foundObsId = i + 1; // ID 1..N
                        break;
                    }
                }

                // Check Map Borders (if point is OUTSIDE borders, treat as obstacle)
                std::vector<Vertex> borderPoly;
                for(const auto& p : map.borders.get_points()) borderPoly.push_back(Vertex(p.x, p.y));
                
                if (!insideObstacle && !PlanningUtils::pointInPolygon(worldPos, borderPoly)) {
                    insideObstacle = true;
                    foundObsId = 0; // ID 0 is Border
                }

                if (insideObstacle) {
                    grid[x][y].dist = 0.0;
                    grid[x][y].sourceObsId = foundObsId;
                    q.push({x, y});
                }
            }
        }

        // 4. Brushfire Algorithm (BFS) to compute Distance Field & Voronoi Regions
        // 8-connected neighbors
        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        double distStep[] = {1.414, 1.0}; // Diagonals cost more

        while (!q.empty()) {
            GridNode current = q.front();
            q.pop();

            PixelInfo& currentInfo = grid[current.x][current.y];

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (!isValid(nx, ny, width, height)) continue;

                PixelInfo& neighborInfo = grid[nx][ny];
                double stepCost = (std::abs(dx[i]) + std::abs(dy[i]) == 2) ? 1.414 : 1.0;
                double newDist = currentInfo.dist + stepCost;

                // If neighbor is unvisited, claim it
                if (neighborInfo.sourceObsId == -1) {
                    neighborInfo.dist = newDist;
                    neighborInfo.sourceObsId = currentInfo.sourceObsId;
                    q.push({nx, ny});
                } 
                // If neighbor is visited by a DIFFERENT obstacle, we found a boundary!
                else if (neighborInfo.sourceObsId != currentInfo.sourceObsId) {
                    // This is a Voronoi Edge (Ridge)
                    // Mark both as Voronoi to ensure connectivity
                    currentInfo.isVoronoi = true;
                    neighborInfo.isVoronoi = true; 
                }
            }
        }

        // 5. Extract Nodes and Build Connectivity
        // Convert 'isVoronoi' pixels to Roadmap Nodes
        
        // We only add nodes if they are sufficiently far from obstacles (clearance threshold)
        // to avoid noisy edges right next to obstacles.
        double minClearance = 1.0 / gridResolution; // e.g. 1 meter

        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                if (grid[x][y].isVoronoi && grid[x][y].dist > minClearance) {
                    Vertex v(minX + x * gridResolution, minY + y * gridResolution);
                    grid[x][y].roadmapNodeIdx = roadmap->addVertex(v);
                }
            }
        }

        // 6. Connect Neighbors
        // For every Voronoi pixel, connect to adjacent Voronoi pixels
        // This creates a grid-like graph along the ridges
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                int uIdx = grid[x][y].roadmapNodeIdx;
                if (uIdx == -1) continue;

                // Check right and bottom neighbors (to avoid duplicate edges)
                // We check 4 neighbors (Right, Bot-Right, Bot, Bot-Left) to cover 8-connectivity
                int cx[] = {1, 1, 0, -1};
                int cy[] = {0, 1, 1, 1};

                for (int i = 0; i < 4; ++i) {
                    int nx = x + cx[i];
                    int ny = y + cy[i];

                    if (isValid(nx, ny, width, height)) {
                        int vIdx = grid[nx][ny].roadmapNodeIdx;
                        if (vIdx != -1) {
                            roadmap->addEdge(uIdx, vIdx, true);
                        }
                    }
                }
            }
        }

        return roadmap;
    }

}