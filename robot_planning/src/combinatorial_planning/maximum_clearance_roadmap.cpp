#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include "combinatorial_planning/planning_utils.h"

namespace MaxClearanceRoadmap {

    struct GridNode { int x, y; };

    struct PixelInfo {
        double dist;       
        int sourceObsId;   
        bool isVoronoi;    
        int roadmapNodeIdx;
    };

    bool isValid(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // --- NEW: Thinning Algorithm (Zhang-Suen) ---
    // This erodes the "thick" Voronoi bands into single-pixel lines
    void applyThinning(std::vector<std::vector<PixelInfo>>& grid, int width, int height) {
        // Neighbors: P2, P3, P4, P5, P6, P7, P8, P9
        // Coordinates: (0,-1), (1,-1), (1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1)
        int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
        int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

        bool pixelRemoved = true;
        while (pixelRemoved) {
            pixelRemoved = false;
            std::vector<GridNode> toRemove;

            // Two sub-iterations required by Zhang-Suen
            for (int iter = 0; iter < 2; ++iter) {
                toRemove.clear();
                for (int x = 1; x < width - 1; ++x) {
                    for (int y = 1; y < height - 1; ++y) {
                        if (!grid[x][y].isVoronoi) continue;

                        int p2 = grid[x + dx[0]][y + dy[0]].isVoronoi;
                        int p3 = grid[x + dx[1]][y + dy[1]].isVoronoi;
                        int p4 = grid[x + dx[2]][y + dy[2]].isVoronoi;
                        int p5 = grid[x + dx[3]][y + dy[3]].isVoronoi;
                        int p6 = grid[x + dx[4]][y + dy[4]].isVoronoi;
                        int p7 = grid[x + dx[5]][y + dy[5]].isVoronoi;
                        int p8 = grid[x + dx[6]][y + dy[6]].isVoronoi;
                        int p9 = grid[x + dx[7]][y + dy[7]].isVoronoi;

                        int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                                (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                                (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                                (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                        int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

                        int m1 = (iter == 0) ? (p2 * p4 * p6) : (p2 * p4 * p8);
                        int m2 = (iter == 0) ? (p4 * p6 * p8) : (p2 * p6 * p8);

                        if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0) {
                            toRemove.push_back({x, y});
                        }
                    }
                }
                
                for (const auto& p : toRemove) {
                    grid[p.x][p.y].isVoronoi = false;
                    pixelRemoved = true;
                }
            }
        }
    }

    std::shared_ptr<Roadmap> maximumClearanceRoadmap(const Map& map, double gridResolution) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        // 1. Determine Grid Dimensions
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        minX -= 1.0; minY -= 1.0; maxX += 1.0; maxY += 1.0; // Padding

        int width = std::ceil((maxX - minX) / gridResolution);
        int height = std::ceil((maxY - minY) / gridResolution);

        if (width <= 0 || height <= 0) return roadmap;

        // 2. Initialize Grid
        std::vector<std::vector<PixelInfo>> grid(width, std::vector<PixelInfo>(height, {std::numeric_limits<double>::max(), -1, false, -1}));
        std::queue<GridNode> q;

        // 3. Rasterize Obstacles (Same logic as before)
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                Vertex worldPos(minX + x * gridResolution, minY + y * gridResolution);
                bool insideObstacle = false;
                int foundObsId = -1;

                const auto& obstacles = map.obstacles.get_obstacles();
                for (size_t i = 0; i < obstacles.size(); ++i) {
                    if (PlanningUtils::pointInObstacle(worldPos, obstacles[i])) {
                        insideObstacle = true; foundObsId = i + 1; break;
                    }
                }
                
                // Check borders
                std::vector<Vertex> borderPoly;
                for(const auto& p : map.borders.get_points()) borderPoly.push_back(Vertex(p.x, p.y));
                if (!insideObstacle && !PlanningUtils::pointInPolygon(worldPos, borderPoly)) {
                    insideObstacle = true; foundObsId = 0;
                }

                if (insideObstacle) {
                    grid[x][y].dist = 0.0;
                    grid[x][y].sourceObsId = foundObsId;
                    q.push({x, y});
                }
            }
        }

        // 4. Brushfire (Same logic as before)
        int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        
        while (!q.empty()) {
            GridNode current = q.front(); q.pop();
            PixelInfo& currentInfo = grid[current.x][current.y];

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (!isValid(nx, ny, width, height)) continue;
                PixelInfo& neighborInfo = grid[nx][ny];

                double stepCost = (std::abs(dx[i]) + std::abs(dy[i]) == 2) ? 1.414 : 1.0;
                
                if (neighborInfo.sourceObsId == -1) {
                    neighborInfo.dist = currentInfo.dist + stepCost;
                    neighborInfo.sourceObsId = currentInfo.sourceObsId;
                    q.push({nx, ny});
                } 
                else if (neighborInfo.sourceObsId != currentInfo.sourceObsId) {
                    // Mark candidate ridge pixels
                    currentInfo.isVoronoi = true;
                    neighborInfo.isVoronoi = true; 
                }
            }
        }

        // --- Apply Thinning ---
        applyThinning(grid, width, height);


        // 5. Extract Nodes (With Sparsity Check)
        double minClearance = 1.0 / gridResolution; 

        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                if (grid[x][y].isVoronoi && grid[x][y].dist > minClearance) {
                    Vertex v(minX + x * gridResolution, minY + y * gridResolution);
                    grid[x][y].roadmapNodeIdx = roadmap->addVertex(v);
                }
            }
        }

        // 6. Connect Neighbors
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                int uIdx = grid[x][y].roadmapNodeIdx;
                if (uIdx == -1) continue;

                // Check 8 neighbors now, as thinning might leave diagonal connections
                for (int i = 0; i < 8; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    if (isValid(nx, ny, width, height)) {
                        int vIdx = grid[nx][ny].roadmapNodeIdx;
                        // Avoid self-loops and double edges (check index order)
                        if (vIdx != -1 && uIdx < vIdx) {
                            roadmap->addEdge(uIdx, vIdx, true);
                        }
                    }
                }
            }
        }

        return roadmap;
    }
}